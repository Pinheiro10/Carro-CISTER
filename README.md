# Carro-CISTER
Código necessário para carro autonomo cister:

os 2 ficheiros incluidos vieram da pasta catkin_new/src/image_processing/scripts

Inicializando:

·         Abrir um terminal com roscore;

Depois dentro o workspace, home/catin_ws  ( ou catkin_new) abrir a câmera com o comando:

·         Source devel/setup.bash

·         Roslaunch zed_wrapper zed.launch;

Dentro do workspace, autoracer/autoracer_ws começar a comunicação com a  Teensy:

·         Source devel/setup.bash

·         Rosrun rosserial_python serial_node.py /dev/ttyACM1 neste Código pode ser um 0 ou 1 no final;
(autoracer/autoracer_ws)  Abrir o talker.py com o comando:

                Rosrun race talker.py

Abrir o kill.py com o comando:

                Rosrun race kill.py

                                Backspace para andar

                                Delete para parar
                              
Dentro do workspace, catkin_new abrir o Line Detection com o comando:

                Rosrun image_processing lane_lines_detection_final.py
                o "lane_lines_detection_final" é a ultima versao

                (Deve abrir a imagem lida se DEBUG ativado)

E abrir o Line Decision também dentro do mesmo workspace:

                Rosrun image_processing line_decision.py

                (workspace, catkin_new)                              
                             
