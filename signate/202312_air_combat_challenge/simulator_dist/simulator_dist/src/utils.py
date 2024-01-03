import random
from .fight import fight
from sys import getsizeof, stderr
from itertools import chain
from collections import deque
try:
    from reprlib import repr
except ImportError:
    pass


def total_size(o, handlers={}, verbose=False):
    """ Returns the approximate memory footprint an object and all of its contents.

    Automatically finds the contents of the following builtin containers and
    their subclasses:  tuple, list, deque, dict, set and frozenset.
    To search other containers, add handlers to iterate over their contents:

        handlers = {SomeContainerClass: iter,
                    OtherContainerClass: OtherContainerClass.get_elements}

    """
    dict_handler = lambda d: chain.from_iterable(d.items())
    all_handlers = {tuple: iter,
                    list: iter,
                    deque: iter,
                    dict: dict_handler,
                    set: iter,
                    frozenset: iter,
                   }
    all_handlers.update(handlers)     # user handlers take precedence
    seen = set()                      # track which object id's have already been seen
    default_size = getsizeof(0)       # estimate sizeof object without __sizeof__

    def sizeof(o):
        if id(o) in seen:       # do not double count the same object
            return 0
        seen.add(id(o))
        s = getsizeof(o, default_size)

        for typ, handler in all_handlers.items():
            if isinstance(o, typ):
                s += sum(map(sizeof, handler(o)))
                break
        return s
    return sizeof(o)


def make_match_info(red:dict, blue:dict, movie:bool, movie_dir:str, time_out:float, common_dir:str, control_init:bool, make_log:bool, visualize:bool)->dict:
    match_info = {
        'teams':{
            'Blue': blue,
            'Red': red
        },
        'visualize': False,
        'time_out': time_out,
        'common_dir': common_dir,
        'registered':{
            'Blue': False,
            'Red': False
        }
    }
    if movie:
        match_info['log_dir'] = movie_dir
        match_info['replay'] = True
        if visualize:
            match_info['visualize'] = 'God'

    match_info['control_init'] = control_init
    match_info['make_log'] = make_log

    return match_info


def run_fight(match_info: dict, random_test:bool)->tuple:
    if random_test:
        return random.choice(['Red', 'Blue', '']), {'Red':1.0, 'Blue':0.0}
    else:
        winner, detail = fight(match_info)
        return winner, detail
