""" Assist Module """

import numpy as np

def assist_command(active_cmd, delayed_cmd, target_pos=None, alpha=0.6, beta=0.3):
    """
    Blend human intention with delayed execution
    alpha → how much autonomy helps (0 = none, 1 = full)
    beta: target bias weight
    """
    active = np.array(active_cmd)
    delayed = np.array(delayed_cmd)

    assisted = alpha * active + (1 - alpha) * delayed

    if target_pos is not None:
        target_vec = np.array(target_pos) - assisted
        assisted = assisted + beta * target_vec  # move slightly toward target

    return assisted.tolist()