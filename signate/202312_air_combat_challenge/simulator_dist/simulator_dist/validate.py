from src.validator import Validator
from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--agent-id', type=str, default = '0')
    parser.add_argument('--agent-module-path', default = './Agent')
    parser.add_argument('--benchmark-id', type=str, default='1')
    parser.add_argument('--benchmark-module-path', default = './BenchMark')
    parser.add_argument('--common-dir', default = './common')

    parser.add_argument('--result-dir', default = './results')
    parser.add_argument('--result-path', default = 'validation_results.json')
    parser.add_argument('--movie', default = 0, type=int)
    parser.add_argument('--num-validation', default=3, type=int)
    parser.add_argument('--time-out', default=0.5, type=float)
    parser.add_argument('--memory-limit', default=3.5, type=float)
    parser.add_argument('--random-test', default=0, type=int)
    parser.add_argument('--control-init', default=1, type=int)
    parser.add_argument('--make-log', default=1, type=int)
    parser.add_argument('--max-size', default=0.3, type=float)
    parser.add_argument('--visualize', default=0, type=int)
    parser.add_argument('--color', default='Red', type=str)

    return parser.parse_args()


def main():
    args = parse_args()
    validator = Validator(agent_id=args.agent_id,
                          agent_module_path=args.agent_module_path,
                          benchmark_id=args.benchmark_id,
                          benchmark_module_path=args.benchmark_module_path,
                          common_dir=args.common_dir
    )
    validator.validate(result_dir=args.result_dir,
                       result_path=args.result_path,
                       movie=args.movie,
                       num_validation=args.num_validation,
                       time_out=args.time_out,
                       memory_limit=args.memory_limit,
                       random_test=args.random_test,
                       control_init=args.control_init,
                       make_log = args.make_log,
                       max_size=args.max_size,
                       visualize=args.visualize,
                       color = args.color
    )


if __name__ == '__main__':
    main()