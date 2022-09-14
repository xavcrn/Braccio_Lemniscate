#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector>
#include <fcntl.h>
#include <dirent.h>

#include "ros/ros.h"
#include "braccio/creation.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

using namespace std;

#define PORT 4242
#define SERIAL_PORT "/dev/ttyACM1"
#define MOVE_DIR "/home/poppy/catkin_ws/src/braccio/moves/jouables"

//#define DEBUG1
//#define DEBUG2
//#define DEBUG3
#define DEBUG4

bool Sequence = false;
int client_sd;
string success("success");
string error("error");

void Free_sequence(std_msgs::Bool retour){
    #ifdef DEBUG2
    ROS_INFO("remote_server : Free_sequence :");
        if(retour.data){
            ROS_INFO("remote_server : Received True");
        } else {
            ROS_INFO("remote_server : Received False");
        }
    #endif
    Sequence = retour.data;
}

class Braccio_robot{
    private:
        int serial_port; // port série controlant la base
        char* serial_port_s;       
        ros::Publisher* egg_control;
        ros::Publisher* player_pub;
        ros::Publisher* recording;

    public:
        vector<string> mouvements;
        int init();
        Braccio_robot(char* _serial_port, ros::Publisher* pl_pub, ros::Publisher* egg_pub, ros::Publisher* _recording);
        ~Braccio_robot();
        int pilotage(char commande[]);
        int creation(char commande[]);
        int pause();
        void sleep();
};

Braccio_robot::~Braccio_robot(){
    sleep();  
}

void Braccio_robot::sleep(){
    #ifdef DEBUG1
    ROS_INFO("remote_server : Braccio is going to sleep");
    #endif
    pause();
    std_msgs::String dodo;
    dodo.data = "sleep";
    #ifndef DEBUG1
    Sequence = true;
    #endif
    player_pub->publish(dodo);
    
    while(Sequence){
        // Read the data received by the subscribers
        ros::spinOnce();
    }
    ROS_INFO("remote_server : Braccio is sleeping");
}

int Braccio_robot::pause(){
    char initialisation[7] = {1, 0,0, 0,0, 0, 0};
    return pilotage(initialisation);
}

Braccio_robot::Braccio_robot(char* _serial_port, ros::Publisher* pl_pub, ros::Publisher* egg_pub, ros::Publisher* _recording){
    serial_port_s = _serial_port;
    player_pub = pl_pub;
    egg_control = egg_pub;
    recording = _recording;
}

int Braccio_robot::init(){
    int res;
    serial_port = open(serial_port_s, O_RDWR | O_NONBLOCK);
    if(serial_port < 0){
        ROS_ERROR("remote_server : Failed opening serial port %s",serial_port_s);
        res = -2;
    }

    res = pause(); // mise en pause des moteurs

    if(res >= 0){ // s'il n'y a pas d'erreur
        // Demander la mise en position initiale du bras
        std_msgs::String msg;
        msg.data = "initial";        
        #ifndef DEBUG1
        Sequence = true;
        #endif
        player_pub->publish(msg);
        #ifdef DEBUG2
        ROS_INFO("remote_server : Publish : \"%s\"   Waiting for answer",msg.data.c_str());
        ROS_INFO("remote_server : spinOnce");
        #endif
                
        // Attendre la terminaison du mouvement
        while(Sequence){
            ros::spinOnce();
        }
    }
    
    // charger la liste des mouvements depuis le dossier correspondant
    DIR *dir = opendir(MOVE_DIR);    
    if(dir != nullptr){
        struct dirent *diread;
        while((diread = readdir(dir)) != nullptr){
            if(diread->d_name[0] != '.'){
                mouvements.push_back(diread->d_name);
            }
        }
        closedir(dir);
    } else {
        ROS_ERROR("remote_server : Error opening directory : %s",MOVE_DIR);
        return -1;
    }
    
    return res;
}

int Braccio_robot::creation(char commande[]){
    ROS_INFO("remote_server : Creating a new movement named : \"%s\"",commande + 2);
    string new_move(commande + 2);
    std_msgs::String rec;
    rec.data = new_move.data();
    //Envoyer la commande à pypot de créer un nouveau mouvement 
    static string recording_str("recording");
    write(client_sd, recording_str.data(), recording_str.length());
    recording->publish(rec);
    char buf[5];
    while(true){
        read(client_sd, buf, 5);
        if(buf[0] == 'S' && buf[1] == 'T' && buf[2] == 'O' && buf[3] == 'P'){
            break;
        }
    }
    #ifdef DEBUG4
    cout << "\"STOP\" received from client" << endl;
    #endif
    rec.data = "STOP";
    recording->publish(rec);
    #ifdef DEBUG4
    cout << "\"STOP\" sentS to pypot" << endl;
    #endif
    string result("success");
    mouvements.push_back(new_move);
    write(client_sd, result.data(), result.length());
    return 0;
}

int Braccio_robot::pilotage(char commande[]){
    // bool permettant de détecter un passage à false sur Sequence
    static bool run = Sequence;
    // messages de controle à envoyer
    static string running("running");
    static string done("done");

    // Envoyer la commande à l'arduino
    int res = write(serial_port, commande + 1, 4);

    if(res < 0){
        ROS_ERROR("remote_server : Base motors lost");
        string msg("Base motors lost");
        write(client_sd, msg.data(), msg.length());
    }

    std_msgs::Bool controlOeuf;
    if(commande[5]){
        controlOeuf.data = true;
    } else {
        controlOeuf.data = false;
    }
    egg_control->publish(controlOeuf);

    // Commandes de séquences
    if(run == true){
        ros::spinOnce();
    }
    if(run == true && Sequence == false){ // Sequence est redescendu à false
        #ifdef DEBUG3
        ROS_INFO("remote_server : Sequence done");
        #endif
        write(client_sd, done.data(), done.length());
        #ifdef DEBUG3
        ROS_INFO("remote_server : Done sent");
        #endif
        run = false;
    }
    else if(run == true && Sequence == true){
        write(client_sd, running.data(), running.length());
    }
    else if(Sequence == false && commande[6] > 0){
        Sequence = true;
        run = true;
        std_msgs::String msg;
        msg.data = mouvements[commande[6]-1].data();
        #ifdef DEBUG1
        ROS_INFO("remote_server : Publishing");
        #endif
        player_pub->publish(msg);
        #ifdef DEBUG1
        ROS_INFO("remote_server : Published");
        #endif
        write(client_sd, running.data(), running.length());
    }
    return res;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "remote_server");
    ros::NodeHandle node;
    
    // Initialisation socket serveur
    int server_sd;
    struct sockaddr_in server_sock;
    server_sock.sin_family = AF_INET;
    server_sock.sin_port = htons(PORT);
    server_sock.sin_addr.s_addr = INADDR_ANY;
    server_sd = socket(AF_INET, SOCK_STREAM, 0);
    int essais = 1;
    while(server_sd < 0){
        if(essais++ > 50){
            ROS_ERROR("Impossible de créé la socket server.");
            exit(-1);
        }
        ROS_ERROR("remote_server : Erreur n°%02d lors de la création de la socket",essais);
        ROS_ERROR("remote server : Nouvelle tentative dans 3 secondes");
        ros::Duration(3.0).sleep();
        server_sd = socket(AF_INET, SOCK_STREAM, 0);
        if(server_sd >= 0){
            ROS_INFO("remote_server : Socket créée");
        }
    }
    /*if(server_sd < 0){
        ROS_ERROR("remote_server : Erreur lors de la création de la socket");
        exit(-1);
    }*/
    essais = 1;
    while(bind(server_sd, (struct sockaddr*) &server_sock, sizeof(server_sock)) < 0){
        if(essais++ > 50){
            ROS_ERROR("remote_server : Impossible de bind, vérifiez que le port %d est libre",PORT);
            exit(-1);
        }
        ROS_ERROR("remote_server : Erreur n°%02d lors du bind",essais);
        ROS_ERROR("remote server : Nouvelle tentative dans 3 secondes");
        ros::Duration(3.0).sleep();
    }
    if(essais > 1){
        ROS_INFO("remote_server : bind réussi");
    }
    /*if(bind(server_sd, (struct sockaddr*) &server_sock, sizeof(server_sock)) < 0){
        ROS_ERROR("remote_server : Erreur lors du bind");
        exit(-1);
    }*/
    listen(server_sd, 2);

    // Prépare une socket client
    struct sockaddr_in client_sock;
    unsigned int client_sock_length;
    client_sock_length = sizeof(client_sock);
    
    string initialized("initialized");
    string off("turned_off");
    
    braccio::creation createur_serveur;
    ros::Publisher player_pub          = node.advertise<std_msgs::String>("play_move", 5);
    ros::Subscriber player_sub         = node.subscribe("movement_playing", 5, Free_sequence);
    ros::Publisher egg_control         = node.advertise<std_msgs::Bool>("egg_activation", 5);
    ros::Publisher recording           = node.advertise<std_msgs::String>("recording",5);

    bool end = false;

    while(ros::ok()){
        ROS_INFO("remote_server : Waiting for a new connection on port %d",PORT);
        client_sd = accept(server_sd, (struct sockaddr*) &client_sock, &client_sock_length);
        ROS_INFO("remote_server : Connection accepted");

        if(client_sd < 0){
            ROS_ERROR("remote_server : Erreur sur accept");
            break;
        }
        char recep[256];
        char envoi[256];
        int n;

        // Envoie un message pour indiquer le succès de la connexion
        write(client_sd, success.data(), success.length());

        Braccio_robot braccio_robot(SERIAL_PORT, &player_pub, &egg_control, &recording);

        ROS_INFO("remote_server : Initializing Braccio");

        if(braccio_robot.init() < 0){
            ROS_ERROR("remote_server : Erreur lors de l'initialisaiton de Braccio");
            write(client_sd, error.data(), error.length());
            close(client_sd);
            break;
        }

        ROS_INFO("remote_server : Braccio initialized");

        write(client_sd, initialized.data(), initialized.length());

        char nb_mouv = braccio_robot.mouvements.size();
        
        ROS_INFO("remote_server : %d movement(s) found",nb_mouv);

        // Récupère le nombre de mouvements enregistrés
        write(client_sd, &nb_mouv, 1);
        // Envoie la liste des mouvements enregistrés
        char buf[2];
        for(int k = 0; k < braccio_robot.mouvements.size(); k++){
            write(client_sd, (char*) braccio_robot.mouvements[k].data(), braccio_robot.mouvements[k].length());
            read(client_sd, buf, 2);
        }

        ROS_INFO("remote_server : Begin control loop");

        bool run = true;
        while(ros::ok() && run){
            n = read(client_sd, recep, sizeof(recep));
            if(n < 0){ // Problème de connexion, mise en pause de la base (par sécurité)
                braccio_robot.pause();
            } else {
                switch(recep[0]){
                    case 0 : // ordre de pause
                        if(recep[1] == 1 && recep[2] == 2 && recep[3] == 3){
                            ROS_INFO("remote_server : Pause");
                            braccio_robot.pause();
                        }
                        break;
                    case 1 : // pilotage
                        braccio_robot.pilotage(recep);
                        break;
                    case 2 : // creation
                        braccio_robot.creation(recep);
                        #ifdef DEBUG4
                        cout << "creation finished" << endl;
                        #endif
                        break;
                    case 3 : // fin
                        ROS_INFO("remote_server : Connection closed by client. Braccio is going to sleep.");
                        run = false;
                        break;
                    default :
                        run = false;
                        break;
                }
            }
        }        
        write(client_sd, off.data(), off.length());
        close(client_sd);
    }
    close(server_sd);
    ROS_INFO("remote_server : Control close");
}
