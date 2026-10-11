import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def run(arguments, output=None, input_data=None, affinity=None):
    command = [str(item) for item in arguments]
    if affinity is not None:
        command = ['taskset', '-c', str(affinity)] + command
    if output is None:
        result = subprocess.run(command, cwd=ROOT, input=input_data, check=True, capture_output=True)
        if result.stdout:
            sys.stdout.buffer.write(result.stdout)
            sys.stdout.buffer.flush()
        return result.stdout
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('wb') as handle:
        subprocess.run(command, cwd=ROOT, input=input_data, stdout=handle, check=True)
    return output

def programs(args):
    folder = (ROOT / args.build).resolve()
    def binary(name):
        path = folder / name
        if not path.is_file():
            raise FileNotFoundError('Build the CMake targets before running this command: ' + str(path))
        return path
    return binary

def copy_study(data, out, name):
    target = out / 'studies' / name
    shutil.copytree(data / 'learning' / name, target, dirs_exist_ok=True)
    return target

def diagnostic_inputs(data, out):
    target = out / 'diagnostic'
    target.mkdir(parents=True, exist_ok=True)
    for path in sorted((data / 'diagnostic').iterdir()):
        if path.name.endswith('.csv.gz'):
            (target / path.name[:-3]).write_bytes(gzip.decompress(path.read_bytes()))
        elif path.suffix == '.csv':
            shutil.copyfile(path, target / path.name)
    return target

def summarize(args, binary, data, out):
    run([binary('main_results'), data / 'main/event_theta5.csv', data / 'main/ga.csv',
         data / 'main/ours_periodic.csv', data / 'main/event_theta0.csv', data / 'main/event_theta2.csv'], out / 'main.json')
    for name, stem, seed in [('initial', 'dqn', 82000), ('followup', 'dqn2', 84000)]:
        folder = data / 'learning' / name
        arguments = [binary(stem + '_results'), '--analyze']
        arguments += [folder / f'test/dqn-{seed + index}.csv' for index in range(5)]
        arguments += [folder / ('test/' + method + '.csv') for method in ('sequential_greedy', 'paper2017_ls', 'paper2017_ls_lk')]
        run(arguments, out / ('learning-' + name + '.json'))
    run([binary('peak_results'), data / 'peak/runs.csv'], out / 'peak.json')
    table = out / 'tables'
    table.mkdir(parents=True, exist_ok=True)
    run([binary('appendix_tables'), out / 'learning-initial.json', out / 'peak.json', table])
    followup = copy_study(data, out, 'followup')
    shutil.copyfile(out / 'learning-followup.json', followup / 'test/summary.json')
    run([binary('dqn_followup_report'), followup, table / 'dqn2-results-table.tex', out / 'learning-followup-budget.json'])
    final = copy_study(data, out, 'final')
    run([binary('dqn3_report'), 'report', final, table / 'dqn3-results-table.tex'])
    run([binary('dqn3_rl_only_report'), 'report', final, table / 'dqn3-rl-only-table.tex'])
    shutil.copyfile(final / 'test/summary.json', out / 'learning-final.json')
    shutil.copyfile(final / 'rl-only-test/summary.json', out / 'learning-rl-only.json')
    diagnostic = diagnostic_inputs(data, out)
    run([binary('dqn_validation_diagnostic'), 'report', diagnostic, table / 'dqn-diagnostic-table.tex'])
    omission(binary, data, out)
    manifest = data / 'timing/inputs.json'
    rows = json.loads(manifest.read_text())['sources']
    run([binary('timing_records'), '--aggregate', manifest] + [data / 'timing/cells' / (Path(row['path']).name[:-9] + '.json') for row in rows], out / 'timing.json')
    run([binary('scale_table'), data / 'scale/checks.csv', table / 'scale-table.tex'])
    run([sys.executable, ROOT / 'scripts/format_tables.py', table])

def omission(binary, data, out):
    target = out / 'omission'
    target.mkdir(parents=True, exist_ok=True)
    with (target / 'counts.csv').open('wb') as handle:
        handle.write(b'floors,cars,rate,seed,total_nonempty,enumerated,enumerated_positive,excluded,excluded_positive\n')
        for source in sorted((data / 'omission/inputs').glob('*.json.gz')):
            result = subprocess.run([str(binary('excluded_count'))], input=gzip.decompress(source.read_bytes()), capture_output=True, check=True)
            handle.write(result.stdout)
    run([binary('excluded_count'), '--aggregate', target / 'counts.csv', data / 'omission/audit.json'], target / 'summary.json')

def main_grid(args, binary, out):
    target = out / 'main'
    target.mkdir(parents=True, exist_ok=True)
    common = [binary('confirmation'), 'grid', '--floors', '8,10,12', '--cars', '2,3,4,5,6',
              '--rates', '10,15,20,25,30', '--seed-from', '50000', '--seeds', '30', '--threads', str(args.jobs),
              '--button-info', '1', '--check', '1', '--duration', '3600', '--max-time', '14400']
    for theta in (0, 2, 5):
        run(common + ['--theta', str(theta), '--methods', 'paper2017,paper2017_lk,paper2017_ls,paper2017_ls_lk,sequential_greedy',
                      '--out', target / f'event_theta{theta}.csv'])
    run(common + ['--theta', '5', '--period', '0.25', '--methods', 'ga2022_all', '--ga-pop', '30', '--ga-gens', '30',
                  '--ga-route', 'collective', '--out', target / 'ga.csv'])
    run(common + ['--theta', '5', '--period', '0.25', '--methods', 'paper2017_ls_lk', '--out', target / 'ours_periodic.csv'])

def concurrent(commands, jobs):
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        for future in [pool.submit(run, command) for command in commands]:
            future.result()

def train(args, binary, out):
    study = args.study
    target = out / 'learning' / study
    target.mkdir(parents=True, exist_ok=True)
    if study == 'final':
        runner = binary('dqn3_runner')
        for auxiliary in ('0', '0.1'):
            run([runner, 'train', 85500, 6500000, 6000000, auxiliary, 900, 90, target / ('tuning/aux-' + auxiliary)])
        selected = run([binary('dqn3_report'), 'choose', target / 'tuning/aux-0/summary.json', target / 'tuning/aux-0.1/summary.json']).decode().strip()
        commands = [[runner, 'train', 86000 + index, 8000000 + index * 100000, 7000000 + index * 100000,
                     selected, 3600, 180, target / f'model-{86000 + index}'] for index in range(5)]
    else:
        initial = study == 'initial'
        runner = binary('dqn_runner' if initial else 'dqn2_runner')
        report = binary('dqn_results' if initial else 'dqn2_results')
        tune_seed, block, episodes, interval = (81000, 1000000, 180, 45) if initial else (83000, 4000000, 600, 180)
        for rate in ('0.001', '0.0003'):
            run([runner, 'train', tune_seed, block, rate, episodes, interval, target / ('tuning/lr-' + rate)])
        selected = run([report, '--choose', target / 'tuning/lr-0.001/summary.json', target / 'tuning/lr-0.0003/summary.json']).decode().strip()
        main_seed, main_block, total, step = (82000, 2000000, 1800, 90) if initial else (84000, 5000000, 3600, 180)
        commands = [[runner, 'train', main_seed + index, main_block + index * 100000, selected, total, step,
                     target / f'model-{main_seed + index}'] for index in range(5)]
    concurrent(commands, args.jobs)

def evaluate(args, binary, data, out):
    study = args.study
    models = Path(args.models).resolve() if args.models else data / 'learning' / study
    target = out / 'evaluations' / study
    shutil.copytree(models, target, dirs_exist_ok=True)
    (target / 'test').mkdir(parents=True, exist_ok=True)
    if study == 'final':
        runner = binary('dqn3_runner')
        commands = []
        old_models = Path(args.previous_models).resolve() if args.previous_models else data / 'learning/followup'
        run([binary('dqn3_rl_only_report'), 'choose', target])
        for index in range(5):
            seed, previous = 86000 + index, 84000 + index
            commands += [[runner, 'test', seed, target / f'model-{seed}/best.bin', target / f'test/dqn3-{seed}.csv'],
                         [runner, 'test', seed, target / f'model-{seed}/checkpoint-0.bin', target / f'test/warm-{seed}.csv'],
                         [runner, 'old2', previous, old_models / f'model-{previous}/best.bin', target / f'test/dqn2-{previous}.csv']]
        methods = ('route_rule', 'sequential_greedy', 'paper2017_ls', 'paper2017_ls_lk')
        commands += [[runner, 'comparators', method, target / ('test/' + method + '.csv')] for method in methods]
        concurrent(commands, args.jobs)
        run([binary('dqn3_report'), 'report', target, target / 'tables.tex'])
        with (target / 'rl-only-test/selected-models.tsv').open() as handle:
            chosen = list(csv.DictReader(handle, delimiter='\t'))
        concurrent([[runner, 'test', row['seed'], row['path'], target / ('rl-only-test/rl-' + row['seed'] + '.csv')] for row in chosen], args.jobs)
        run([binary('dqn3_rl_only_report'), 'report', target, target / 'rl-only-tables.tex'])
    else:
        initial = study == 'initial'
        stem, start = ('dqn', 82000) if initial else ('dqn2', 84000)
        runner = binary(stem + '_runner')
        commands = [[runner, 'test', start + index, target / f'model-{start + index}/best.bin', target / f'test/dqn-{start + index}.csv'] for index in range(5)]
        methods = ('sequential_greedy', 'paper2017_ls', 'paper2017_ls_lk')
        commands += [[runner, 'comparators', method, target / ('test/' + method + '.csv')] for method in methods]
        concurrent(commands, args.jobs)
        run([binary(stem + '_results'), '--analyze'] + [target / f'test/dqn-{start + index}.csv' for index in range(5)]
            + [target / ('test/' + method + '.csv') for method in methods], target / 'test/summary.json')

def timing(args, binary, data, out):
    target = out / 'timing'
    target.mkdir(parents=True, exist_ok=True)
    manifest = data / 'timing/inputs.json'
    results = []
    for row in json.loads(manifest.read_text())['sources']:
        source = ROOT / row['path']
        if args.data != 'data':
            source = data / 'timing/inputs' / source.name
        tokens = subprocess.run([str(binary('timing_records')), '--tokens'], input=gzip.decompress(source.read_bytes()), capture_output=True, check=True).stdout
        result = target / (source.name[:-9] + '.json')
        run([binary('structured_dispatch_probe'), '30'], result, tokens, args.cpu)
        results.append(result)
    run([binary('timing_records'), '--aggregate', manifest] + results, target / 'summary.json')
    record = {'platform': sys.platform, 'logical_cpus': os.cpu_count(), 'selected_logical_cpu': args.cpu,
              'timer': 'std::chrono::steady_clock', 'frequency_controlled': False, 'repetitions': 30}
    (target / 'environment.json').write_text(json.dumps(record, indent=2) + '\n')

def diagnostic(args, binary, data, out):
    target = out / 'diagnostic-rerun'
    target.mkdir(parents=True, exist_ok=True)
    runner = binary('dqn_validation_diagnostic')
    commands = [[runner, 'model', seed, data / f'learning/followup/model-{seed}/best.bin', target, 'validation-only'] for seed in range(84000, 84005)]
    commands += [[runner, 'rule', method, target] for method in ('raw_eta', 'insertion')]
    concurrent(commands, args.jobs)
    run([runner, 'report', target, target / 'tables.tex'])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=('summarize', 'main', 'peak', 'omission', 'timing', 'scale', 'train', 'evaluate', 'diagnostic', 'smoke'))
    parser.add_argument('study', nargs='?', default='final', choices=('initial', 'followup', 'final'))
    parser.add_argument('--build', default='build')
    parser.add_argument('--data', default='data')
    parser.add_argument('--out', default='outputs')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--models')
    parser.add_argument('--previous-models')
    parser.add_argument('--cpu', type=int)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    binary = programs(args)
    data = (ROOT / args.data).resolve()
    out = (ROOT / args.out).resolve()
    if out == data or data in out.parents:
        parser.error('choose an output directory outside the packaged data')
    out.mkdir(parents=True, exist_ok=True)
    if args.command == 'summarize':
        summarize(args, binary, data, out)
    elif args.command == 'main':
        main_grid(args, binary, out)
    elif args.command == 'peak':
        run([binary('peak_runner'), out / 'peak.csv'])
        run([binary('peak_results'), out / 'peak.csv'], out / 'peak.json')
    elif args.command == 'omission':
        omission(binary, data, out)
    elif args.command == 'timing':
        timing(args, binary, data, out)
    elif args.command == 'scale':
        run([binary('scale_check'), out / 'scale.csv'])
        run([binary('scale_table'), out / 'scale.csv', out / 'scale.tex'])
    elif args.command == 'train':
        train(args, binary, out)
    elif args.command == 'evaluate':
        evaluate(args, binary, data, out)
    elif args.command == 'diagnostic':
        diagnostic(args, binary, data, out)
    elif args.command == 'smoke':
        run([binary('egcs'), 'run', '--floors', '10', '--cars', '4', '--rate', '10', '--seed', '50000',
             '--method', 'paper2017_ls_lk', '--theta', '5', '--button-info', '1', '--check', '1', '--duration', '60', '--max-time', '14400'])
        run([binary('dqn3_runner'), 'pilot', out / 'learning-pilot'])

if __name__ == '__main__':
    main()
