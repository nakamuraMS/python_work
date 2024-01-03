import os
import json
import tracemalloc

from .utils import make_match_info, run_fight, total_size


class Validator():
    def __init__(self,
                 agent_id:str,
                 agent_module_path:str,
                 benchmark_id:str,
                 benchmark_module_path:str,
                 common_dir: str
        ) -> None:

        self.agent_id = agent_id
        self.agent_module_path = agent_module_path
        self.benchmark_module_path = benchmark_module_path

        self.agent_info = _get_agent_info(agent_id, agent_module_path)
        self.benchmark_info = _get_agent_info(benchmark_id, benchmark_module_path)

        self.common_dir = common_dir


    def validate(self,
                 result_dir: str,
                 result_path: str,
                 movie: bool,
                 num_validation: int,
                 time_out: float,
                 memory_limit: float,
                 random_test: bool,
                 control_init: bool,
                 make_log: bool,
                 max_size: float,
                 visualize: bool,
                 color: str
        ) -> None:
        print('Validation:')
        print('random test: {}\n'.format(random_test))

        # fight against the benchmark
        results_dir = os.path.join(result_dir, self.agent_id)
        results = self._fight_benchmark(results_dir, movie, time_out, num_validation, memory_limit, random_test, control_init, make_log, visualize, color)
        os.makedirs(results_dir, exist_ok=True)
        for i, log in results['details'].items():
            s = total_size(log['logs'][color])
            if s <= max_size*(1024**3):
                print('Log size for fight {}: {} [GB]'.format(i, s/(1024**3)))
                with open(os.path.join(results_dir, 'log_{}.json'.format(i)), 'w', encoding='utf-8') as f:
                    json.dump(log['logs'][color], f, indent=1)
            else:
                print('Log size for fight {} is too large: {} [GB](>={}[GB])'.format(i+1, s/(1024**3), max_size))
            
            with open(os.path.join(results_dir, 'config_{}.json'.format(i)), 'w', encoding='utf-8') as f:
                json.dump(log['config'], f, indent=1)
            del log['logs'], log['config']
        results_path = os.path.join(results_dir, result_path)
        with open(results_path, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=1)
        print('\nSaved the result to {}'.format(results_path))


    def _fight_benchmark(self, results_dir:str, movie:bool, time_out:float, num_validation:int, memory_limit:float, random_test:bool, control_init:bool, make_log: bool, visualize: bool, color: str)->dict:
        memory_limit = memory_limit * (1024**3)
        if color=='Red':
            print('You are Red')
            red = self.agent_info
            blue = self.benchmark_info
            color_opponent = 'Blue'
        elif color == 'Blue':
            print('You are Blue')
            red = self.benchmark_info
            blue = self.agent_info
            color_opponent = 'Red'
        else:
            print('You are Red')
            red = self.agent_info
            blue = self.benchmark_info
            color_opponent = 'Blue'

        match_info = make_match_info(red = red,
                                     blue = blue,
                                     movie = movie,
                                     movie_dir = results_dir,
                                     time_out = time_out,
                                     common_dir = self.common_dir,
                                     control_init = control_init,
                                     make_log = make_log,
                                     visualize = visualize
        )

        details = {}
        tracemalloc.start()
        num_wins, num_losses, num_draw = 0, 0, 0
        for i in range(num_validation):
            print('\n---fight {}---'.format(i+1))
            winner, detail = run_fight(match_info, random_test)
            snapshot = tracemalloc.take_snapshot()
            stats = snapshot.statistics('lineno')
            total_memory = 0
            for stat in stats:
                total_memory += stat.size
            print('fight {}: {} [Bytes] so far'.format(i+1, total_memory))
            if total_memory >= memory_limit:
                print('total memory: {} [Bytes](>={})'.format(total_memory, memory_limit))
                raise MemoryLimitExceed('Total memory exceeded the limit.')
            tracemalloc.clear_traces()
            if detail['error']:
                raise ActionError('Action error.')
            details[i+1] = detail
            if winner == color:
                num_wins += 1
            elif winner == color_opponent:
                num_losses += 1
            elif winner == '':
                num_draw += 1
            print('fight {}: Red {} vs Blue {}, winner {}'.format(i+1, red['userModelID'], blue['userModelID'], winner))

        results = {
            'own':color,
            'details': details,
            'win':num_wins,
            'loss':num_losses,
            'draw':num_draw,
            'nocontest': 0
        }

        return results


def _get_agent_info(user_agent:str, module_path:str)->dict:
    module_name = os.path.split(module_path)[-1]
    with open(os.path.join(module_path, 'args.json')) as f:
        args = json.load(f)
    agent_info = {
        'userModelID': user_agent,
        'userModuleID': module_name,
        'args': args
    }
    return agent_info


class MemoryLimitExceed(Exception):
    pass


class ActionError(Exception):
    pass