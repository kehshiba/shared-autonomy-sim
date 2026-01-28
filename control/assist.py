import numpy as np

def assist_command(active_cmd, delayed_cmd,alpha=0.6):
    """
    Blend human intention with delayed execution
    alpha → how much autonomy helps (0 = none, 1 = full)
    """
    active = np.array(active_cmd)
    delayed=np.array(delayed_cmd)

    assisted = alpha * active + ( 1 - alpha ) * delayed

    return assisted.tolist()