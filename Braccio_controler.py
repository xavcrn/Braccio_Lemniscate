from numpy import uint8, uint8
import pygame
import socket
from time import sleep

#Definition des couleurs
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

#Definition des boutons
R_joy_y = 3
L_joy_y = 1
RB      = 5
LB      = 4
RT      = 7
LT      = 6
R_joy_b = 11
L_joy_b = 10
START   = 9
BACK    = 8
A_b     = 2
B_b     = 1


FPS = 30

# Protocole de communication avec Braccio :
# Premier octet : code de commande
#   0 -> commande de pause
#   1 -> commande de mouvement
#   2 -> commande de creation de mouvement

#Classe permettant d'afficher du texte a l'ecran
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

pygame.init()

screen = pygame.display.set_mode((800, 450))

pygame.display.set_caption("Braccio conttroler")

done  = False
close = False
error = False

clock = pygame.time.Clock()

pygame.joystick.init()

textPrint = TextPrint()

cpt = 0

#Connection de la manette
while not done:
    screen.fill(WHITE)
    textPrint.reset()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()

    if(pygame.joystick.get_count() == 0):
        textPrint.tprint(screen, "Branchez une manette sur votre ordinateur")
    elif(pygame.joystick.get_count() > 1):
        textPrint.tprint(screen, "Ne branchez qu'une seule manette")
    else:
        textPrint.tprint(screen, "Manette connectee")
        textPrint.tprint(screen, "Connexion a Braccio..")
        done = True
    
    pygame.display.flip()
    clock.tick(FPS)
joystick = pygame.joystick.Joystick(0)

#Connection a Braccio
#"""#debug
braccio = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
braccio.connect(("braccio.local", 4242))
data = braccio.recv(256)

if data != b'success':
    print("Erreur de communication avec Braccio")
    print("Recu : <{}>".format(data))
    error = True

if not error:
    screen.fill(WHITE)
    textPrint.reset()
    textPrint.tprint(screen, "Initialisation de Braccio...")
    pygame.display.flip()
    clock.tick(FPS)

    data = braccio.recv(256)
    if data != b'initialized':
        print("Erreur d'initialisation, recu <{}>".format(data))
        error = True
    #"""#debug

    #recuperer la liste des mouvements pre-enregistres
    mouvements = []
    print("Recuperation de la liste des mouvements enregistres")
    n = int.from_bytes(braccio.recv(1),"little") #recuperation du nombre de mouvements enregistres
    print("{} mouvement(s) disponible(s)".format(n))
    for k in range(n):
        mouvements.append(braccio.recv(256))
        braccio.send(b'ok')
        print(mouvements[k])

    #Menu principal
    selection = 0
    selectionnable = ["Piloter Braccio","Enregistrer un nouveau mouvement","Quitter"]
    #"""#debug
    int_msg_list = [uint8(0),uint8(1),uint8(2),uint8(3)]
    braccio.send(bytes(int_msg_list)) # Pause order
    #"""#debug

#Creation de mouvements
def creation_mouvement():
    global close
    fin_creation = False
    nom = ""
    creation = False
    msg = ""
    cpt = 0
    curseur = "|"
    while not fin_creation and not close:
        if not creation:
            for event in pygame.event.get():            
                if event.type == pygame.QUIT:
                    close = True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if joystick.get_button(BACK) == 1:
                        fin_creation = True
                    elif joystick.get_button(START) == 1:
                        if len(nom) == 0:
                            msg = "Veuillez entrer un nom pour votre nouveau mouvement"
                        else:
                            creation = True
                            msg_to_send = bytearray(b'\x02')
                            msg_to_send.append(60)
                            nom_bytes = bytes(nom, 'ASCII')
                            for k in nom_bytes:
                                msg_to_send.append(k)
                            msg_to_send.append(0)
                            braccio.send(msg_to_send)
                            msg = "Creation en cours du mouvement \"" + nom + "\""
                            
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_BACKSPACE:
                        nom = nom[:-1]
                    else:
                        nom += event.unicode
        else:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if joystick.get_button(START) == 1:
                        braccio.send(b'STOP')
                        data = braccio.recv(256)
                        fin_creation = True
                        msg = "Le mouvement \"" + nom + "\" a ete creer avec succes"
                        mouvements.append(nom)

        # fait clignoter la barre
        cpt += 1
        if cpt == 16:
            curseur = ""
        elif cpt == 32:
            curseur = "|"
            cpt = 0

        screen.fill(WHITE)
        textPrint.reset()        
        textPrint.tprint(screen, "Creation de mouvement")
        textPrint.tprint(screen, "")
        textPrint.tprint(screen, "Appuyez sur START pour confirmer le nom et lancer l'enregistrement")
        textPrint.tprint(screen, "Appuyez à nouveau sur START pour terminer le mouvement")
        textPrint.tprint(screen, "Appuyez sur BACK pour quitter")
        textPrint.tprint(screen, "")
        textPrint.tprint(screen, "Entrez le nom du mouvement que vous voulez enregistrer :")
        textPrint.tprint(screen, nom + curseur)
        textPrint.tprint(screen, msg)
        
        pygame.display.flip()
        clock.tick(FPS)
    sleep(1)

#Pilotage du robot
def pilotage():
    moteur_droit  = uint8(0)
    moteur_gauche = uint8(0)
    sens_droit    = uint8(0)
    sens_gauche   = uint8(0)
    controlOeuf   = uint8(0)
    joueSequence  = uint8(0)
    global close
    fin_pilotage = False
    selection = 0
    jouer_mouvement = False
    while not close and not fin_pilotage:
        screen.fill(WHITE)
        textPrint.reset()
        textPrint.tprint(screen, "Pilotage en cours")
        textPrint.tprint(screen, "Selectionnez un mouvement avec LB et RB, executez-le avec stick_L + stick_R")
        textPrint.tprint(screen, "retour avec BACK, (des)activez le controle via les oeufs avec RT + LT")
        textPrint.tprint(screen, "")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                close = True
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(BACK) == 1:
                    fin_pilotage = True
                elif joystick.get_button(RT) == 1 and joystick.get_button(LT) == 1 :
                    if controlOeuf == uint8(0):
                        controlOeuf = uint8(1)
                    else :
                        controlOeuf = uint8(0)
                elif len(mouvements) != 0 and not jouer_mouvement:
                    if controlOeuf == 0 and joystick.get_button(R_joy_b) == 1 and joystick.get_button(L_joy_b) == 1:
                        jouer_mouvement = True
                        joueSequence = uint8(selection + 1)
                    elif joystick.get_button(LB) == 1:
                        if selection == 0:
                            selection = len(mouvements) - 1
                        else :
                            selection -= 1
                    elif joystick.get_button(RB) == 1:
                        if selection == len(mouvements) -1:
                            selection = 0
                        else :
                            selection += 1
        
        joyR = joystick.get_axis(R_joy_y)
        joyL = joystick.get_axis(L_joy_y)

        msgG = "Moteur gauche :" 
        msgD = "Moteur droit  :"

        if joyL > 0:
            sens_droit    = 0
            moteur_droit  = uint8(255*joyL)
            msgD += " -"
        else :
            sens_droit    = 1
            moteur_droit  = uint8(-255*joyL)
            msgD += "  "
        if joyR > 0:
            sens_gauche   = 0
            moteur_gauche = uint8(255*joyR)
            msgG += " -"
        else :
            sens_gauche   = 1
            moteur_gauche = uint8(-255*joyR)
            msgG += "  "

        if moteur_droit  < 15:
            moteur_droit  = uint8(0)
        if moteur_gauche < 15:
            moteur_gauche = uint8(0)

        if fin_pilotage == 1:
            moteur_droit  = uint8(0)
            moteur_gauche = uint8(0)

        msgG += "{}".format(moteur_gauche)
        msgD += "{}".format(moteur_droit)    

        textPrint.tprint(screen, msgG)
        textPrint.tprint(screen, msgD)
        
        textPrint.tprint(screen, "")
        if controlOeuf == uint8(0) :
            textPrint.tprint(screen, "Oeufs : desactives")
        else :
            textPrint.tprint(screen, "Oeufs : actives")
        textPrint.tprint(screen, "")
        
        #Affichage des mouvement preenregistres
        textPrint.tprint(screen, "Liste des mouvements enregistres :")
        textPrint.indent()
        if len(mouvements) != 0:            
            for k in range(0,len(mouvements)):
                if selection == k:
                    textPrint.indent()
                    if jouer_mouvement:
                        textPrint.tprint(screen, str(mouvements[k])[2:-1] + " <-- en cours")
                    else:
                        textPrint.tprint(screen, mouvements[k])
                    textPrint.unindent()
                else :
                    textPrint.tprint(screen, mouvements[k])
        else :
            textPrint.tprint(screen, "Vous n'avez enregistre aucun mouvements.")
            textPrint.tprint(screen, "Utilisez \"Enregistrer un mouvement\"")

        #Envoie des controles
        #"""#debug
        msg_to_send = [uint8(1),sens_gauche,moteur_gauche,sens_droit,moteur_droit,controlOeuf,joueSequence] # uint8(1) is to tell Braccio it is a command order
        braccio.send(bytes(msg_to_send))
        #"""#debug
        if jouer_mouvement:
            if joueSequence != 0:
                joueSequence = 0
            data = braccio.recv(256)
            if data == b'done':
                jouer_mouvement = False

        #refresh display
        pygame.display.flip()

        #limits to 30 frames per second
        clock.tick(FPS)


while not error and not close:
    screen.fill(WHITE)
    textPrint.reset()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            close = True
        if event.type == pygame.JOYBUTTONDOWN:
            if joystick.get_button(START) == 1:
                if selection == 0:
                    pilotage()
                    #"""#debug
                    msg_to_send = [uint8(0),uint8(1),uint8(2),uint8(3)]
                    braccio.send(bytes(msg_to_send)) # Pause order
                    #"""#debug
                elif selection == 1:
                    creation_mouvement()
                    #"""#debug
                    msg_to_send = [uint8(0),uint8(1),uint8(2),uint8(3)]
                    braccio.send(bytes(msg_to_send)) # Pause order
                    #"""#debug
                else :
                    close = True
                screen.fill(WHITE)
                textPrint.reset()
            elif joystick.get_button(RB) == 1:
                if selection == len(selectionnable) -1:
                    selection = 0
                else : 
                    selection += 1
            elif joystick.get_button(LB) == 1:
                if selection == 0:
                    selection = len(selectionnable) -1
                else :
                    selection -= 1

    textPrint.tprint(screen, "Menu Principal")
    textPrint.tprint(screen, "")
    textPrint.tprint(screen, "Selectionnez une action avec LB et RB, vaidez avec START")
    textPrint.tprint(screen, "")
    for k in range(0,3):
        if selection == k:
            textPrint.indent()
            textPrint.tprint(screen, selectionnable[k])
            textPrint.unindent()
        else :
            textPrint.tprint(screen, selectionnable[k])
    
    pygame.display.flip()

    clock.tick(FPS)
    
#Extinction de Braccio
#"""#debug
while not error and data != b'turned_off':
    screen.fill(WHITE)
    textPrint.reset()
    textPrint.tprint(screen, "Extinction de Braccio...")
    pygame.display.flip()
    clock.tick(FPS)
    braccio.send(uint8(3))
    data = braccio.recv(256)

braccio.close()
#"""#debug

screen.fill(WHITE)
textPrint.reset()
textPrint.tprint(screen, "Braccio eteint")
pygame.display.flip()

sleep(1)

pygame.quit()
