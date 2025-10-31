import yasmin
import rclpy
import yasmin
import json
from yasmin import State, CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from utbots_actions.action import TextToSpeech, Transcription, InterpretNLU
from std_msgs.msg import String

class Person():
    def __init__(self, name=None, drink=None, interested_in=None):
        self.name = name
        self.drink = drink
        self.interested_in = interested_in
        self.done = False

    def get_done(self):
        return self.done
    
    def set_done(self, done):
        self.done = done
    
    def __str__(self):
        return self.name

    def __repr__(self):
        return f"Person(name={self.name})"

class SendTTSState(ActionState):
    def __init__(self) -> None:
         super().__init__(
            TextToSpeech,  # action type
            "/tts",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TextToSpeech.Goal:
        goal = TextToSpeech.Goal()
        goal.request.text = blackboard["text"]
        return goal

def whisper_process_cb(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """
    if blackboard["whispered"] == "" or blackboard["whispered"] is None:
        return "process_whisper1"#return to whisper

    return "process_whisper2"#pass to NLU


class WhisperSTTState(ActionState):
    """
    Class representing the state of the Whisper action.

    Inherits from ActionState and implements methods to handle the
    Whisper action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self) -> None:
        """
        Initializes the WhisperTTSState.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            Transcription,  # action type
            "/utbots/transcription",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Transcription.Goal:
        """
        Creates the goal for the Whisper action.

        This mblackboard["tts_text"]ethod retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = Transcription.Goal()
        # goal.order = blackboard["n"]  # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: Transcription.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["whispered"] = (
            response.text.data
        )  # Store the result sequence in the blackboard
        print(blackboard["whispered"])
        return SUCCEED

def whisper_print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Whisper action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
    return SUCCEED


class CoquiTTSState(ActionState):
    """
    Class representing the state of the coqui-TTS action.

    Inherits from ActionState and implements methods to handle the
    TTS action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self) -> None:
        """
        Initializes the CoquiTTSState.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            TextToSpeech,  # action type
            "/utbots/tts",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TextToSpeech.Goal:
        """
        Creates the goal for the Whisper action.

        This method retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = TextToSpeech.Goal()
        text = blackboard["tts_text"]
        # data = String()
        if (text is not None):
            goal.text.data = blackboard["tts_text"]  # Retrieve the input value 'n' from the blackboard
        else:
            goal.text.data =  "Hi my name is Hestia. I have nothing to say in the moment." # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: TextToSpeech.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        return SUCCEED


def coqui_print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Whisper action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
    return SUCCEED

class NLUInference (ActionState):
    """
    Class representing the state of the rasa-NLU action.

    Inherits from ActionState and implements methods to handle the
    NLU action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self,verbose=False) -> None:
        """
        Initializes the NLUInference.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        self.verbose = verbose
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO("NLUInference initialized")
        super().__init__(
            InterpretNLU,  # action type
            "/utbots/interpret_nlu",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> InterpretNLU.Goal:
        """
        Creates the goal for the Whisper action.

        This method retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = InterpretNLU.Goal()
        text = blackboard["nlu_input_text"]
        # data = String()
        if (text is not None):
            goal.nlu_input.data = blackboard["nlu_input_text"]  # Retrieve the input value 'n' from the blackboard
        else:
            goal.nlu_input.data =  "" # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: InterpretNLU.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["nlu_output"] = (
            response.nlu_output.data
        )  # Store the result sequence in the blackboard

        blackboard["nlu_intent"] = (
            response.task.data
        )  # Store the result sequence in the blackboard

        blackboard["nlu_data"] = (
            response.data.data
        )  # Store the result sequence in the blackboard
        blackboard["nlu_chat"] = (
            response.bot_response.data
        )  # Store the result sequence in the blackboard
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO(f"NLU Output: {blackboard['nlu_output']}")
            yasmin.YASMIN_LOG_INFO(f"NLU Intent: {blackboard['nlu_intent']}")
            yasmin.YASMIN_LOG_INFO(f"NLU Data: {blackboard['nlu_data']}")
            yasmin.YASMIN_LOG_INFO(f"NLU Chat Response: {blackboard['nlu_chat']}")
        return SUCCEED

    def print_result(blackboard: Blackboard) -> str:
        """
        Prints the result of the Whisper action.

        This function logs the final result stored in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard containing the result.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
        return SUCCEED

def wait_cb(blackboard: Blackboard) -> str:
    from time import sleep
    sleep(1)
    return "waited"

PROCESS_NLU=[ 
    "greet",                # 0
    "introduce_robot",      # 1
    "affirm",               # 2
    "deny",                 # 3
    "mood_great",           # 4
    "mood_unhappy",         # 5
    "follow",               # 6
    "stop",                 # 7
    "go_to",                # 8
    "say_operator_name",    # 9
    "identify_operator",    # 10
    "describe_ambient",     # 11
    "like_drink",           # 12
    "pick_object",          # 13
    "ask_interested_in",    # 14
    "default",              # 15
    "ask_about_bahia",      # 16
    "bahia_facts",          # 17  
    "bahia_geography",      # 18
    "bahia_climate",        # 19
    "bahia_economy",        # 20
    ]             

def get_process_nlu():
    return PROCESS_NLU

class NLUProcess(CbState):
    """
    Class representing the NLU process.

    This class contains methods to handle the NLU process in a finite state machine.

    Attributes:
        None
    """
    def __init__(self, verbose=False) -> None:
        """
        Initializes the NLUInference.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            outcomes=PROCESS_NLU, cb=self.nlu_process_cb
        )
        

        self.verbose = verbose

    def _parse_entities(self, entities_json_string):
        """Parse entities from JSON string."""
        try:
            entities = json.loads(entities_json_string)
            return entities if isinstance(entities, list) else []
        except (json.JSONDecodeError, TypeError, AttributeError):
            if self.verbose:
                print(f"Failed to parse entities JSON: {entities_json_string}")
            return []
    
    def _get_first_entity_value(self, entities):
        """Get the value of the first entity (for backward compatibility)."""
        if entities and len(entities) > 0:
            return entities[0].get('value')
        return None
    
    def _get_entity_values_by_type(self, entities, entity_type):
        """Get all entity values of a specific type."""
        return [entity.get('value') for entity in entities 
                if entity.get('entity') == entity_type and entity.get('value')]
    
    def _get_highest_confidence_entity_value(self, entities, entity_type=None):
        """Get the value of the entity with the highest confidence score."""
        filtered_entities = entities
        if entity_type:
            filtered_entities = [e for e in entities if e.get('entity') == entity_type]
        
        if not filtered_entities:
            return None
            
        best_entity = max(filtered_entities, key=lambda x: x.get('confidence_entity', 0))
        return best_entity.get('value')
    
    def _get_entities_by_type(self, entities, entity_type):
        """Get all entities of a specific type (full entity objects)."""
        return [entity for entity in entities if entity.get('entity') == entity_type]

    def nlu_process_cb(self,blackboard: Blackboard) -> str:
        """
        Retrieves the next waypoint from the list of random waypoints.

        Updates the blackboard with the pose of the next waypoint.

        Args:
            blackboard (Blackboard): The blackboard instance holding current state data.

        Returns:
            str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
                navigation is complete (END).
        """

        task=blackboard["nlu_intent"]
        
        # Parse entities from JSON string
        entities = self._parse_entities(blackboard["nlu_data"])
        
        # Extract specific entity values
        data = self._get_first_entity_value(entities)
        topics = self._get_entity_values_by_type(entities, "topic")
        names = self._get_entity_values_by_type(entities, "name")
        drinks = self._get_entity_values_by_type(entities, "drink")
        locations = self._get_entity_values_by_type(entities, "location")

        # Store all entities in blackboard for later use
        blackboard["nlu_entities"] = entities
        blackboard["nlu_topics"] = topics
        blackboard["nlu_names"] = names
        blackboard["nlu_drinks"] = drinks
        blackboard["nlu_locations"] = locations
        
        answer=None
        # Use the parsed entities more effectively
        name = names[0] if names else data  # Use first name entity or fallback to data
        drink = drinks[0] if drinks else data  # Use first drink entity or fallback to data
        location = locations[0] if locations else data  # Use first location entity or fallback to data
        topic = topics[0] if topics else data  # Use first topic entity or fallback to data
        
        # For cases where you need to handle multiple topics
        if topics and len(topics) > 1:
            topics_str = ", ".join(topics[:-1]) + " and " + topics[-1]
        else:
            topics_str = topic
        
        outcome=None
        match task:
            case "greet":
                answer="Hello, my name is hestia!How are you?"
                outcome="greet"
            case "introduce_robot":
                answer="Hello, my name is hestia!Im a service robot designed by the Utbots team, from Curitiba, Brazil. \
                    (bla bla bla)."
                outcome="introduce_robot"
            case "affirm":
                answer="The operator affirmed my request."
                outcome="affirm"
            case "deny":
                answer="The operator denied my request."
                outcome="deny"
            case "mood_great":
                answer="Amazing!"
                outcome="mood_great"
            case "mood_unhappy":
                answer="Oh thats sad!"
                outcome="mood_unhappy"
            case "follow":
                answer="I was asked to follow the operator.Wait for follow mode to start."
                outcome="follow"
            case "stop":
                answer="I was asked to stop! Stopping."
                outcome="stop"
            case "go_to":
                answer=f"I was asked to go to the {location}!Navigation starting."
                blackboard["waypoint_nametag"]=location
                outcome="go_to"
            case "say_operator_name":
                # answer=f"I was asked to say the operators name!The name is {name}."
                answer= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
                blackboard["name"]= name           
                outcome="say_operator_name"
            case "identify_operator":
                # answer=f"I was asked to say the operators name!The name is {name}."
                answer= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
                blackboard["name"]= name           
                outcome="identify_operator"
            case "describe_ambient":
                answer=f"I was asked to describe the ambient {location}!"
                outcome="describe_ambient"
            case "like_drink":
                answer=f"The operator likes {drink}, does he?Please deny or affirm the sentence."
                outcome="describe_ambient"
                blackboard["drink"]= drink 
            case "pick_object":
                answer=f"I was asked to pick {data}!Please deny or confirm the request."
                outcome="describe_ambient"
            case "ask_about_bahia":
                answer="You asked about Bahia. What would you like to know?"
                outcome="ask_about_bahia"
            case "bahia_facts":
                answer=f"Here are some facts about Bahia!"
                outcome="bahia_facts"
            case "bahia_geography":
                answer=f"Bahia is located in northeastern Brazil..."
                outcome="bahia_geography"
            case "bahia_climate":
                answer=f"Bahia has a tropical climate..."
                outcome="bahia_climate"
            case "bahia_economy":
                answer=f"Bahia's economy is based on..."
                outcome="bahia_economy"
            case "ask_interested_in":
                answer=blackboard["nlu_chat"]
                blackboard["interested_in"]=topics_str
                outcome="ask_interested_in"
            case _:  # Default case
                answer="Hello, my name is hestia!I wasn't able to understand what you said to me!"
                outcome="default"
        blackboard["tts_text"]=answer
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO(f"NLU Intent: {task}")
            yasmin.YASMIN_LOG_INFO(f"NLU Data: {data}")
            yasmin.YASMIN_LOG_INFO(f"Entities: {entities}")
            yasmin.YASMIN_LOG_INFO(f"Names: {names}")
            yasmin.YASMIN_LOG_INFO(f"Drinks: {drinks}")
            yasmin.YASMIN_LOG_INFO(f"Locations: {locations}")
            yasmin.YASMIN_LOG_INFO(f"Topics: {topics}")
            yasmin.YASMIN_LOG_INFO(f"Answer: {answer}")
            yasmin.YASMIN_LOG_INFO(f"Outcome: {outcome}")
        blackboard["nlu_data"] = data
        return outcome

class Register(CbState):
    """
    Class representing the NLU process.

    This class contains methods to handle the NLU process in a finite state machine.

    Attributes:
        None
    """
    def __init__(self, 
                 verbose=False, 
                #  limit=3
                 ) -> None:
        """
        Initializes the NLUInference.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            outcomes=[SUCCEED,CANCEL], 
            cb=self.register_person_cb
        )
        self.verbose = verbose

    def register_person_cb(self,blackboard: Blackboard) -> str:
        """
        Retrieves the next waypoint from the list of random waypoints.

        Updates the blackboard with the pose of the next waypoint.

        Args:
            blackboard (Blackboard): The blackboard instance holding current state data.

        Returns:
            str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
                navigation is complete (END).
        """

        try:
            new_person = {
                "name": blackboard["name"],
                "drink": blackboard["drink"],
                "interested_in": blackboard["interested_in"]
            }

            blackboard["person_list"].append(new_person)

            if(self.verbose):
                yasmin.YASMIN_LOG_INFO(f"New person registered: {new_person}")
            outcome=SUCCEED

        except:
            if(self.verbose):
                yasmin.YASMIN_LOG_INFO("Failed to register new person.")
            outcome=CANCEL
        
        return outcome


def generate_ask_name_sm():
    ask_name_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    ask_name_sm.add_state(
        "ASK_SOMETHING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: ABORT,
        },
        remappings={"tts_text":"ask_name"}
    )

    ask_name_sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_name_sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
            # "process_whisper3": "outcome4",

        },
    )

    ask_name_sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_name_sm.add_state(
        "NLU_PROCESS",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: "ASK_SOMETHING",
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "VERIFY",
            PROCESS_NLU[10]: "VERIFY",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
            PROCESS_NLU[15]: "ASK_SOMETHING",
            PROCESS_NLU[16]: "ASK_SOMETHING",
            PROCESS_NLU[17]: "ASK_SOMETHING",
            PROCESS_NLU[18]: "ASK_SOMETHING",
            PROCESS_NLU[19]: "ASK_SOMETHING",
        },
    )

    ask_name_sm.add_state(
        "VERIFY",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER_VER",
            CANCEL: ABORT,
        },
    )

    ask_name_sm.add_state(
        "CALLING_WHISPER_VER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_name_sm.add_state(
        "WHISPER_PROCESS_VER",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER_VER",
            "process_whisper2": "NLU_INFERENCE_VER",
            # "process_whisper3": "outcome4",

        },
    )

    ask_name_sm.add_state(
        "NLU_INFERENCE_VER",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_name_sm.add_state(
        "NLU_PROCESS_VER",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: SUCCEED,
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
            PROCESS_NLU[15]: "ASK_SOMETHING",
            PROCESS_NLU[16]: "ASK_SOMETHING",
            PROCESS_NLU[17]: "ASK_SOMETHING",
            PROCESS_NLU[18]: "ASK_SOMETHING",
            PROCESS_NLU[19]: "ASK_SOMETHING",
        },
    )
    return ask_name_sm

def generate_ask_drink_sm():
    ask_drink_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    ask_drink_sm.add_state(
        "ASK_SOMETHING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: ABORT,
        },
        remappings={"tts_text":"ask_drink"}
    )

    ask_drink_sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
            # "process_whisper3": "outcome4",

        },
    )

    ask_drink_sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_drink_sm.add_state(
        "NLU_PROCESS",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: "ASK_SOMETHING",
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "VERIFY",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
            PROCESS_NLU[15]: "ASK_SOMETHING",
            PROCESS_NLU[16]: "ASK_SOMETHING",
            PROCESS_NLU[17]: "ASK_SOMETHING",
            PROCESS_NLU[18]: "ASK_SOMETHING",
            PROCESS_NLU[19]: "ASK_SOMETHING",
        },
    )

    ask_drink_sm.add_state(
        "VERIFY",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER_VER",
            CANCEL: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "CALLING_WHISPER_VER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "WHISPER_PROCESS_VER",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER_VER",
            "process_whisper2": "NLU_INFERENCE_VER",
            # "process_whisper3": "outcome4",

        },
    )

    ask_drink_sm.add_state(
        "NLU_INFERENCE_VER",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_drink_sm.add_state(
        "NLU_PROCESS_VER",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: SUCCEED,
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
            PROCESS_NLU[15]: "ASK_SOMETHING",
            PROCESS_NLU[16]: "ASK_SOMETHING",
            PROCESS_NLU[17]: "ASK_SOMETHING",
            PROCESS_NLU[18]: "ASK_SOMETHING",
            PROCESS_NLU[19]: "ASK_SOMETHING",
        },
    )
    return ask_drink_sm


def ask_interested_in_sm():
    ask_interested_in_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    ask_interested_in_sm.add_state(
        "ASK_SOMETHING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: ABORT,
        },
        remappings={"tts_text":"ask_interested_in"}
    )

    ask_interested_in_sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_interested_in_sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
        },
    )

    ask_interested_in_sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_interested_in_sm.add_state(
        "NLU_PROCESS",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: SUCCEED,
            PROCESS_NLU[1]: SUCCEED,
            PROCESS_NLU[2]: SUCCEED,
            PROCESS_NLU[3]: SUCCEED,
            PROCESS_NLU[4]: SUCCEED,
            PROCESS_NLU[5]: SUCCEED,
            PROCESS_NLU[6]: SUCCEED,
            PROCESS_NLU[7]: SUCCEED,
            PROCESS_NLU[8]: SUCCEED,
            PROCESS_NLU[9]: SUCCEED,
            PROCESS_NLU[10]: SUCCEED,
            PROCESS_NLU[11]: SUCCEED,
            PROCESS_NLU[12]: SUCCEED,
            PROCESS_NLU[13]: SUCCEED,
            PROCESS_NLU[14]: SUCCEED,
            PROCESS_NLU[15]: SUCCEED,
            PROCESS_NLU[16]: SUCCEED,
            PROCESS_NLU[17]: SUCCEED,
            PROCESS_NLU[18]: SUCCEED,
            PROCESS_NLU[19]: SUCCEED,
            PROCESS_NLU[20]: SUCCEED,
        },
    )

    return ask_interested_in_sm