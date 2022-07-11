#include <socket>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>

#include "ros/ros.h"
#include "braccio/creation.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

using namespace std;

#define PORT 4242
#define SERIAL_PORT "/dev/ttyACM1"
#define MOVE_DIR "./move/"

class Braccio_robot(){
    private:
        int serial_port; // port série controlant la base
        ros::NodeHandle* n;
        ros::ServiceClient createur_client;
        braccio::creation  createur_serveur;
        ros::Publisher joueur_pub;
        ros::Subscriber joueur_sub;
        int client_sd;
        bool sequence; // Une séquence est-elle en cours ?
        void free_sequence();

    public:
        vector<string> mouvements;
        int init();
        braccio_robot(int _client_sd, ros::NodeHandle* _n, string _serial_port);
        ~braccio_robot();
        int pilotage(char commande[]);
        int creation(char commande[]);
        int pause();
        void sleep();
};

Braccio_robot::free_sequence(){
    sequence = false;
}

Braccio_robot::~braccio_robot(){
    sleep();  
}

void Braccio_robot::sleep(){
    pasue();
    std_msgs::String dodo;
    dodo.data = "sleep";
    sequence = true;
    joueur_pub.publish(dodo);
    while(sequence);
}

int Braccio_robot::pause(){
    char initialisation[7] = {1, 0,0, 0,0, 0, 0};
    return pilotage(initialisation);
}

Braccio_robot::braccio_robot(int _client_sd, ros::NodeHandle* _n, char* _serial_port){
    
    client_sd = _client_sd;
    sequence = false;
    n = _n;
    createur_client = n->serviceClient<braccio::creation>("creation");
    joueur_pub      = n->advertise<std_msgs::String>("mouvement",10);
    joueur_sub      = n->subscribe("termine", 10, free_sequence);
}

int Braccio_robot::init(){
    int res = pause(); // mise en pause des moteurs
    if(res){ // en cas d'erreur
        // Demander la mise en position initiale du bras
        std_msgs::String msg;
        msg.append("initial");
        sequence = true;
        joueur_pub.publish(msg);
        // Attendre la terminaison du mouvement
        while(sequence);
        return res;
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
        ROS_ERROR("Error opening directory : %s",MOVE_DIR);
        return -1;
    }
    serial_port = open(_serial_port, O_RDWR);
    if(serial_port < 0){
        ROS_ERROR("Failed opening serial port %s",_serial_port);
        res = -2;
    }
    return res;
}

void Braccio_robot::creation(char commande[]){
    string new_move(commande + 2);    
    //Envoyer la commande à pypot de créer un nouveau mouvement (via un service) 
    createur_serveur.request.move_name = new_move;
    createur_serveur.request.duration = (uint8_t)commande[1];
    createur_client.call(createur_serveur);
    bool success = createur_serveur.response.success;
    string result;
    if(success){
        result.append("success");
        mouvements.push_back(new_move);
    } else {
        result.append("failure");
    }
    write(client_sd, result.data(), result.length());
}

int Braccio_robot::pilotage(char commande[]){
    // Envoyer la commande à l'arduino
    int res = write(serial_port, commande + 1, 4);

    if(res < 0){
        string msg("lose base motors");
        write(client_sd, msg.data(), msg.length());
    }

    bool controlOeuf;
    if(commande[5]){
        controlOeuf = true;
    } else {
        controlOeuf = false;
    }

    if(joueSequence > 0 && sequence == false){
        sequence = true;
        std_msgs::String msg;
        msg.data = mouvements[commande[6]-1].data();
        joueur_pub.publish(msg);
    }

    return res;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "controler_recept");
    ros::NodeHandle node;
    
    // Initialisation socket serveur
    int server_sd;
    struct sockaddr_in server_sock;
    server_sock.sin_family = AF_INET;
    server_sock.sin_port = PORT;
    server_sock.sin_addr.s_addr = INADDR_ANY;
    server_sd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_sd < 0){
        ROS_ERROR("Erreur lors de la création de la socket");
        exit(-1);
    }    
    if(bind(server_sd, (struct sockaddr*) &server_sock, sizeof(server_sock)) == -1){
        ROS_ERROR("Erreur lors du bind");
        exit(-1);
    }
    listen(server_sd, 1);

    // Prépare une socket client
    int client_sd;
    struct sockaddr_in client_sock;
    int client_sock_length;
    client_sock_length = sizeof(client_sock);
    
    string success("success");
    string error("error");
    string initialized("initialized");

    while(ros::ok()){
        client_sd = accept(serveur_sd, (struct sockaddr*) &client_sock, &client_sock_length);
        
        if(client_sd < 0){
            ROS_ERROR("Erreur sur accept");
            continue;
        }
        char recep[256];
        char envoi[256];
        int n;

        // Encoie un message pour indiquer le succès de la connexion
        
        write(client_sd, success.data(), success.length());

        Braccio_robot braccio_robot(client_sd, &node, SERIAL_PORT);

        if(braccio_robot.init() < 0){
            ROS_ERROR("Erreur lors de l'initialisaiton de Braccio");
            write(client_sd, error.data(), error.length());
            close(client_sd);
            continue;
        }

        write(client_sd, initialized.data(), initialized.length());

        // Récupère le nombre de mouvements enregistrés
        write(client_sd, (char) mouvements.size(), 1);
        // Envoie la liste des mouvements enregistrés
        for(int k = 0; k < mouvements.size(); k++){
            write(client_sd, mouvements[k].data(), mouvements[k].length());
        }

        bool run = true;
        while(ros::ok() && run){
            n = read(client_sock, recep, sizeof(recep));
            if(n < 0){ // Problème de connexion, mise en pause de la base (par sécurité)
                braccio_robot.pause();
            } else {
                switch(recep[0]){
                    case 0 : // ordre de pause
                        if(recep[1] == 1 && recep[2] == 2 && recep[3] == 3){
                            braccio_robot.pause();
                        }
                        break;
                    case 1 : // pilotage
                        braccio_robot.pilotage(recep);
                        break;
                    case 2 :
                        braccio_robot.creation(recep);
                        break;
                    case 'C':
                        if(recep[1] == 'L' && recep[2] == 'O' && recep[3] == 'S' && recep[4] == 'E'){
                            run = false;
                            braccio_robot.init();
                        }
                        break;
                    default :
                        run = false;
                        break;
                }
            }
        }
        close(client_sd);
    }
    close(server_sd);
    ROS_INFO("Control close");
}