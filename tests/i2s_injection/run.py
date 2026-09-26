"""Execute real stepper2 stream functions and real I2S sample renderer on host."""
from pathlib import Path
import argparse
import importlib.util
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT/'main/grbl'
spec = importlib.util.spec_from_file_location('direct', CORE/'tests/stepper2/run.py')
direct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(direct)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', required=True)
args = parser.parse_args()
s = (CORE/'stepper2.c').read_text()
start = s.index('typedef enum {')
end = s.index('\n};', s.index('struct st2_motor {'))+3
names = ['st2_motor_config', 'st2_reset', 'st2_profile_set_speed',
         'st2_motor_set_speed', 'st2_profile_start', 'st2_executor_start',
         'st2_motor_move', 'st2_get_position', 'st2_set_position',
         'st2_profile_advance', 'st2_executor_run', 'motor_irq',
         'st2_motor_run', 'st2_profile_request_stop', 'st2_motor_stop',
         'st2_motor_running', 'st2_stream_next', 'st2_stream_progress',
         'st2_stream_service', 'st2_stream_move']
code = '#include "mock_hal.h"\n#include "i2s_injection.h"\n'+s[start:end]+'''
static st2_motor_t *motors;
static void (*on_reset)(void);
static bool st2_stream_move(st2_motor_t *, float, float, position_t);
static void st2_stream_service(st2_motor_t *);
'''
code += '\n'.join(direct.function(s, name) for name in names)
code += '\n'+Path(__file__).with_name('cases.h').read_text()
with tempfile.TemporaryDirectory(prefix='i2s-injection-') as tmp:
    path = Path(tmp)
    (path/'test.c').write_text(code)
    command = [args.cc, '-std=c11', '-Wall', '-DSTEP_INJECT_STREAM=1',
               '-I'+str(CORE/'tests/stepper2'), '-I'+str(CORE), '-I'+str(ROOT/'main'),
               str(path/'test.c'), str(ROOT/'main/i2s_injection.c'), '-o', str(path/'test.exe')]
    if 'tcc' not in Path(args.cc).name.lower():
        command += ['-lm']
    subprocess.run(command, check=True)
    subprocess.run([str(path/'test.exe')], check=True)
