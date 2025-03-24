from itertools import product


def is_invalid_state(state):
    """
    Checks if the given state is invalid based on the constraints.

    :param state: A dictionary representing the state.
    :return: Boolean indicating whether the state is invalid.
    """
    if state["coralHopper"] and state["coralOuttake"]:
        return True
    if not state["reefAlign"] and state["reefDetected"]:
        return True
    if state["reefDetected"] and state["elevatorPos"] not in ["L2", "L3", "L4"]:
        return True
    if state["coralHopper"] and state["elevatorPos"] != "INTAKE":
        return True
    if not state["algaeDetected"] and state["elevatorPos"] in ["AP", "AN"]:
        return True
    if not state["coralOuttake"] and state["elevatorPos"] in ["L1", "L2", "L3", "L4"]:
        return True
    if not state["reefDetected"] and state["elevatorPos"] in ["L2", "L3", "L4"]:
        return True
    if not state["reefAlign"] and state["elevatorPos"] in ["L1"]:
        return True
    if sum([state["reefAlign"], state["netAlign"], state["processorAlign"]]) > 1:
        return True
    if state["netAlign"] and state["elevatorPos"] != "AN":
        return True
    if state["processorAlign"] and state["elevatorPos"] != "AP":
        return True
    if state["elevatorPos"] in ["AN", "AP"] and state["reefAlign"]:
        return True
    
    return False


def get_result_states(state):
    """
    Determines the result states based on the provided state.

    :param state: A dictionary representing the state.
    :return: A list of result states.
    """
    result_states = []

    if (
        not state["coralHopper"]
        and not state["coralOuttake"]
        and state["elevatorPos"] == "INTAKE"
    ):
        result_states.append("CORAL_PREINTAKE")

    if state["coralHopper"] and state["elevatorPos"] == "INTAKE":
        result_states.append("CORAL_TRANSFER")

    if (
        state["coralOuttake"]
        and not state["reefAlign"]
    ):
        result_states.append("CORAL_READY")

    if (
        state["coralOuttake"]
        and state["elevatorPos"] in ["ZERO", "INTAKE"]
        and state["reefAlign"]
        and not state["reefDetected"]
    ):
        result_states.append("CORAL_PRESCORE")

    if (
        state["coralOuttake"] == True
        and state["elevatorPos"] in ["L1", "L2", "L3", "L4"]
        and state["reefDetected"] == True
        and state["reefAlign"] == True
    ):
        result_states.append(f"CORAL_SCORE_{state['elevatorPos']}")

    if (
        state["algaeDetected"] == True
        and not state["processorAlign"] and not state["netAlign"]):
        result_states.append("ALGAE_READY")

    if not state["algaeDetected"] and state["elevatorPos"] in ["A2", "A3"]:
        result_states.append(f"ALGAE_INTAKE_{state['elevatorPos']}")

    if (
        state["algaeDetected"] == True
        and state["processorAlign"]):
        result_states.append("ALGAE_PRESCORE_AP")

    if (
        state["algaeDetected"] == True
        and state["netAlign"]):
        result_states.append("ALGAE_PRESCORE_AN")

    if (
        state["coralOuttake"] == True
        and state["elevatorPos"] == "L1"
        and state["reefAlign"] == True
    ):
        result_states.append(f"CORAL_SCORE_{state['elevatorPos']}")


    if (
        state["coralHopper"] is False
        and state["coralOuttake"] is False
        and state["elevatorPos"] == "ZERO"
        and state["algaeDetected"] is False
    ):
        result_states.append("IDLE")

    return result_states


def get_prioritized_state(result_states):
    """
    Given a list of result states, return the highest-priority state.
    Priority is given to 'score', 'transfer', and 'prescore' states over 'ready' states.

    :param result_states: List of result states.
    :return: The highest-priority state as a string.
    """
    # Priority order (higher priority means earlier in the list)
    priority_order = [
        "CORAL_SCORE_",  # Score states have the highest priority
        "CORAL_TRANSFER", # Transfer comes next
        "CORAL_PRESCORE", # Prescore comes next
        "ALGAE_INTAKE",   # Algae intake
        "ALGAE_PRESCORE_AP", # Prescore for algae with processor alignment
        "ALGAE_PRESCORE_AN", # Prescore for algae with net alignment
        "CORAL_PREINTAKE",
        "CORAL_READY",    # Ready states come after transfer, score, and prescore
        "ALGAE_READY",    # Ready states are of equal priority
        "IDLE"            # Idle state has the lowest priority
    ]
    
    # Sort result states based on priority order
    for state in priority_order:
        for result_state in result_states:
            if state in result_state:
                return result_state
    
    # If no state matches the priority order, return an empty string (or None, depending on your preference)
    return ""

if __name__ == "__main__":
    state_variables = {
        "coralHopper": [True, False],
        "coralOuttake": [True, False],
        "elevatorPos": [
            "ZERO",
            "INTAKE",
            "AP",
            "L1",
            "A2",
            "L2",
            "A3",
            "L3",
            "L4",
            "AN",
        ],
        "reefAlign": [True, False],
        "reefDetected": [True, False],
        "algaeDetected": [True, False],
        "netAlign": [True, False],
        "processorAlign": [True, False]
    }

    keys = state_variables.keys()
    values_combinations = product(*state_variables.values())

    all_states = []
    for values in values_combinations:
        state = dict(zip(keys, values))

        # Initialize validState and resultState
        state["validState"] = ""
        state["resultState"] = [""]

        # Check if the state is invalid
        if is_invalid_state(state):
            state["validState"] = "❌"
        else:
            # Get the result states if valid
            state["resultState"] = get_result_states(state)
            state["resultState"] = get_prioritized_state(state["resultState"])

        all_states.append(state)

    valid_states_count = 0
    invalid_states_count = 0
    null_states_count = 0
    total_states = len(all_states)

    result_states = [str(state["resultState"]) for state in all_states if state["validState"] != "❌"]
    unique_result_states = sorted(list(set(result_states)))
    unique_count = len(unique_result_states)

    for state in all_states:
        if state["validState"] == "❌":
            invalid_states_count += 1
        elif state["resultState"]:
            valid_states_count += 1
        else:
            print(state)
            null_states_count += 1

    with open("notes/stateTable.md", "w", encoding="utf-8") as mdfile:
        mdfile.write(f"Invalid States: {invalid_states_count} / {total_states}\n\n")
        mdfile.write(f"Valid States: {valid_states_count} / {total_states}\n\n")
        mdfile.write(f"Null States: {null_states_count} / {total_states}\n\n")

        mdfile.write(f"Unique Result States: {unique_count}\n\n")
        mdfile.write("\n".join([f"- {state}" for state in unique_result_states]))
        mdfile.write("\n\n")

        headers = " | ".join(
            list(state_variables.keys()) + ["resultState", "validState"]
        )
        separator = " | ".join(["---"] * (len(state_variables) + 2))
        mdfile.write(f"| {headers} |\n")
        mdfile.write(f"| {separator} |\n")
        for state in all_states:
            try:
                row = (
                    " | ".join(str(state[key]) for key in state_variables.keys())
                    + " | "
                    + state["resultState"]
                    + " | "
                    + state["validState"]
                )
            except TypeError:
                row = (
                    " | ".join(str(state[key]) for key in state_variables.keys())
                    + " | "
                    + ""
                    + " | "
                    + state["validState"]
                )
            mdfile.write(f"| {row} |\n")
