from pathlib import Path
import re
import sys

def blocks(root, name, count):
    found = re.findall(r'\\begin\{longtable\}.*?\\end\{longtable\}', (root / name).read_text(), re.S)
    if len(found) != count:
        raise ValueError('Unexpected table count: ' + name)
    return found

def main():
    root = Path(sys.argv[1])
    initial = blocks(root, 'dqn-results-table.tex', 2)
    followup = blocks(root, 'dqn2-results-table.tex', 3)
    final = blocks(root, 'dqn3-results-table.tex', 3)
    rl = blocks(root, 'dqn3-rl-only-table.tex', 1)
    diagnostic = blocks(root, 'dqn-diagnostic-table.tex', 2)
    final = [table.replace('Guided D3QN', 'Pretrained network') for table in final]
    summary = next(line for line in rl[0].splitlines() if line.startswith('RL-only five-model mean &'))
    secondary = 'Reward-trained mean (secondary) & ' + ' & '.join(summary.split(' & ')[2:])
    final[0] = final[0].replace('Sequential insertion &', secondary + '\nSequential insertion &')
    tables = [final[0], rl[0]] + final[1:] + initial + followup + diagnostic
    text = '\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{0}\n'
    text += '\n'.join(tables) + '\n\\endgroup\n'
    (root / 'learning-tables.tex').write_text(text)

if __name__ == '__main__':
    main()
