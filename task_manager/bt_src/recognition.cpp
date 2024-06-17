#include "recognition.hpp"

Recognize::Recognize() : recognition_action("recognition", true){

    ROS_INFO("[BT::RECOGNITION] Waiting for recognition action server to start");
    // wait for the action server to start
    recognition_action.waitForServer();

    ROS_INFO("[BT::RECOGNITION] Recognition server has started");
}

BT::NodeStatus Recognize::Recognition(BT::TreeNode& tree){
    // Chama ação

    utbots_face_recognition::RecognizeGoal goal;

    ROS_INFO("[BT::RECOGNITION] Sending recognition action");

    recognition_action.sendGoalAndWait(goal, ros::Duration(3));
    
    // Change to both get and send image to parameter server (action needs to have sensor_msgs.msg Image as definition, goal is already done)
    utbots_face_recognition::RecognizeResultConstPtr result = recognition_action.getResult();

    // Returns failure if recognition didn't send an image back (aka failed).

    // In the future change it so if a name is repeated within recognized people than it should return failure (aka recognized someone else as an operator)
    try{result->image.data;}
    catch(...){return BT::NodeStatus::FAILURE;}

    // Aqui não faço nada mas tenho que mandar para o parâmetro de variáveis da árvore de alguma forma
    result->image;

    return BT::NodeStatus::SUCCESS;
    
}

NewFace::NewFace(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config), new_face_action("new_face", true) {

    ROS_INFO("[BT::NEW_FACE] Waiting for new face action server to start");
    // wait for the action server to start
    new_face_action.waitForServer();

    ROS_INFO("[BT::NEW_FACE] New face server has started");
    
}

BT::NodeStatus NewFace::onStart(){
    // Chama a ação
    // Retorna running
}

BT::NodeStatus NewFace::onRunning(){
    // Lê o feedback e verifica se acabou/obteve sucesso Log de feedback

    // Lembrar de retornar
}

void NewFace::onHalted(){
    // Para, já pode ficar...
}