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
    if not state["algaeDetected"] and state["elevatorPos"] in ["PROCESS", "NET"]:
        return True
    if not state["coralOuttake"] and state["elevatorPos"] in ["L1", "L2", "L3", "L4"]:
        return True
    if not state["reefDetected"] and state["elevatorPos"] in ["L2", "L3", "L4"]:
        return True
    if not state["reefAlign"] and state["elevatorPos"] in ["L1"]:
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
        and state["elevatorPos"] in ["ZERO", "INTAKE"]
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

    if state["algaeDetected"] == True:
        result_states.append("ALGAE_PRESCORE")

    if not state["algaeDetected"] and state["elevatorPos"] in ["A2", "A3"]:
        result_states.append(f"ALGAE_INTAKE_{state['elevatorPos']}")

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


if __name__ == "__main__":
    state_variables = {
        "coralHopper": [True, False],
        "coralOuttake": [True, False],
        "elevatorPos": [
            "ZERO",
            "INTAKE",
            "PROCESS",
            "L1",
            "A2",
            "L2",
            "A3",
            "L3",
            "L4",
            "NET",
        ],
        "reefAlign": [True, False],
        "reefDetected": [True, False],
        "algaeDetected": [True, False],
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
            state["resultState"] = (
                ", ".join(state["resultState"]) if state["resultState"] else ""
            )

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
