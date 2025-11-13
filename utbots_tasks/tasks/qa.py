import yasmin
import datetime
import rclpy
from fpdf import FPDF
from yasmin import State, CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from std_msgs.msg import String


from utbots_tasks.states.basic_voice import WhisperSTTState,whisper_process_cb

from utbots_tasks.states.basic_voice import LlamaState

from utbots_tasks.states.basic_voice import NLUInference,get_process_nlu,NLUProcess

from utbots_tasks.states.basic_voice import CoquiTTSState

from utbots_tasks.states.basic_voice import wait_cb

def check_hello(blackboard):
    if "hello" in blackboard["whispered"].lower():
        return 'succeeded'
    return 'faliure'

def save_conversation_cb(blackboard):
    """Adiciona a última pergunta e resposta ao histórico no blackboard."""
    

    question = "Pergunta não registrada."
    if "whispered" in blackboard and blackboard["whispered"] is not None:
        question = blackboard["whispered"]

    answer = "Resposta não registrada."
    if "llm_output" in blackboard and blackboard["llm_output"] is not None:
        answer = blackboard["llm_output"]
    

    history = blackboard["qa_history"]
    history.append(f"Usuário: {question}")
    history.append(f"Robô: {answer}")
    

    blackboard["qa_history"] = history
    
    print(f"Conversa salva no histórico. Itens atuais: {len(history)}")
    
    return 'succeeded'
       
def generate_pdf_from_history(blackboard):
    """Pega todo o histórico de conversas do blackboard e gera um arquivo PDF."""
    
    history_list = []

    if "qa_history" in blackboard:
        history_list = blackboard["qa_history"]
    
    if not history_list:
        print("Histórico vazio. Nenhum PDF será gerado.")
        return 'report_generated'

    pdf = FPDF()
    pdf.add_page()
    pdf.set_font('Helvetica', size=12)
    pdf.set_title('Histórico de Perguntas e Respostas')

    now = datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    pdf.set_font('Helvetica', style='B', size=16)
    pdf.cell(0, 10, 'Relatório de Conversa', ln=1, align='C', border=1)
    pdf.set_font('Helvetica', size=10)
    pdf.cell(0, 10, f'Gerado em: {now}', ln=1, align='C')
    pdf.ln(10)

    pdf.set_font('Helvetica', size=12)

    for text_entry in history_list:

        pdf.multi_cell(0, 8, text_entry.encode('latin-1', 'replace').decode('latin-1'))
        pdf.ln(2) 

    pdf.output("qa_report.pdf")
    
    print("PDF 'qa_report.pdf' gerado com sucesso.")
    
    return 'report_updated' 


PROCESS_NLU=get_process_nlu()

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    
    rclpy.init()
    
    node = rclpy.create_node("qa_sm")
    
    qa_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    verbose = True

    set_ros_loggers()

    blackboard = Blackboard()
    blackboard["tts_text"] = None
    blackboard["qa_history"] = []  
    blackboard["text_input"] = None
    blackboard["llm_output"] = None
    blackboard["whispered"] = None
    blackboard["nlu_input_text"] = None
    blackboard["nlu_output"] = None
    blackboard["nlu_intent"] = None  
    blackboard["nlu_data"] = None 
    blackboard["confirm"] = "Ask me a question."

    qa_sm.add_state(
        "WAIT_FOR_ACTIVATION", 
        WhisperSTTState(),
        transitions={SUCCEED: "VERIFY_HELLO", ABORT: ABORT}
   #     transitions={SUCCEED : "NLU_INFERENCE", ABORT:ABORT}
    )
    

    qa_sm.add_state(
        "VERIFY_HELLO",
        CbState(["succeeded", "faliure"], check_hello),
       
        transitions={"succeeded": "CONFIRM_OPERATOR", "faliure" : "WAIT_FOR_ACTIVATION"}, 
   )




    #qa_sm.add_state(
    #    "NLU_INFERENCE",
    #    NLUInference(),
    #    transitions={SUCCEED : "NLU_PROCESS", ABORT:ABORT},
    #    remappings={"nlu_input_text": "whispered"}
    #)
    # PROCESS_NLU=[ "greet",
#     "introduce_robot",
#     "affirm",
#     "deny",
#     "mood_great",
#     "mood_unhappy",
#     "follow",
#     "stop",
#     "go_to",
#     "say_operator_name",
#     "identify_operator",
#     "describe_ambient",
#     "like_drink"
#     "pick_object"
#     "default",]
#    qa_sm.add_state(
#        "NLU_PROCESS",
#        NLUProcess(verbose),
#        remappings={"nlu_input_text" : "whispered"},
#        transitions={
#            PROCESS_NLU[0]: "CONFIRM_OPERATOR",
#            PROCESS_NLU[1]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[2]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[3]: "WAIT/_FOR_ACTIVATION",
#            PROCESS_NLU[4]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[5]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[6]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[7]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[8]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[9]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[10]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[11]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[12]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[13]: "WAIT_FOR_ACTIVATION",
#            PROCESS_NLU[14]: "WAIT_FOR_ACTIVATION"
#        }
#    )

    qa_sm.add_state(
        "CONFIRM_OPERATOR",
        CoquiTTSState(),
        transitions={SUCCEED: "LISTEN_FOR_QUESTION", ABORT:ABORT},
        remappings={"tts_text" : "confirm"}
    )

    qa_sm.add_state(
        "LISTEN_FOR_QUESTION", 
        WhisperSTTState(), 
        transitions={SUCCEED: "SEND_QUESTION_TO_LLM", ABORT: ABORT},
    )

    qa_sm.add_state(
        "SEND_QUESTION_TO_LLM", 
        LlamaState(), 
        transitions={SUCCEED: "SPEAK_LLM_ANSWER", ABORT: ABORT}
    )


    qa_sm.add_state(
        "SPEAK_LLM_ANSWER", 
        CoquiTTSState(), 
        transitions={SUCCEED: "SAVE_CONVERSATION_TO_HISTORY", ABORT: ABORT},
        remappings={"tts_text" : "llm_output"}
    )

    qa_sm.add_state(
        "SAVE_CONVERSATION_TO_HISTORY",
        CbState(["succeeded"], save_conversation_cb),
        transitions={"succeeded": "UPDATE_PDF_REPORT"}, 
    )


    qa_sm.add_state(
        "UPDATE_PDF_REPORT",
        CbState(["report_updated"], generate_pdf_from_history),
        transitions={"report_updated": "WAIT_FOR_ACTIVATION"},
    )


    try:
        outcome = qa_sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if qa_sm.is_running():
            qa_sm.cancel_state()


    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
