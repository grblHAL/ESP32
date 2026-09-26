"""Exercise the real ESP32 submission/claim guards with a completed prior motion."""
import argparse, importlib.util, subprocess, tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('extract',ROOT/'main/grbl/tests/stepper2/run.py')
extract=importlib.util.module_from_spec(spec);spec.loader.exec_module(extract)
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cc',required=True);a=p.parse_args()
driver=(ROOT/'main/i2s_out.c').read_text()
source=r'''
#include <assert.h>
#include <math.h>
#define ceilf(x) ((float)ceil((double)(x)))
#include <stdio.h>
#include "i2s_injection.h"
#define IRAM_ATTR
#define Z_AXIS 2
#define Z_AXIS_BIT 4
#define Z_STEP_PIN 67
#define Z_DIRECTION_PIN 68
#define I2S_OUT_PIN_BASE 64
#define I2S_OUT_USEC_PER_PULSE 4
#define bit(n) (1u<<(n))
enum {PASSTHROUGH, STEPPING, WAITING};
static int i2s_out_pulser_status=PASSTHROUGH;
static bool i2s_out_initialized=true, injection_claimed, injection_cancel_pending;
static bool injection_fault_pending,injection_drain_only;
static i2s_injection_t injection;
static unsigned writes, mode_changes, locks, events;
static struct {struct {struct {bool z;}dir_invert,step_invert;
    float pulse_microseconds,pulse_delay_microseconds;}steppers;} settings;
static void injection_lock(void) {locks++;}
static void injection_unlock(void) {assert(locks);locks--;}
static void i2s_out_set_stepping(void) {mode_changes++;i2s_out_pulser_status=STEPPING;}
static void i2s_out_write(unsigned p,bool v) {(void)p;(void)v;writes++;}
static injection_event_t next(void *ctx) {
    (void)ctx;events++;return (injection_event_t){4,true,true};
}
'''
source+='\n'.join(extract.function(driver,n) for n in
                  ('injection_supports','injection_submit','i2s_injection_claim'))
source+=r'''
int main(void) {
    settings.steppers.pulse_microseconds=8;
    injection_motion_t motion={.id=1,.axis_mask=4,.direction_mask=4,
        .requested_steps=1,.next=next};
    assert(i2s_injection_claim(Z_AXIS,true));
    assert(injection_submit(&motion));
    // Complete a previous correction through real render/EOF accounting.
    uint32_t samples[16]={0};i2s_injection_checkpoint_t cp={.generation=1};
    injection_progress_t progress;
    assert(i2s_injection_render(&injection,samples,16,&cp));
    assert(i2s_injection_confirm(&injection,&cp,1,&progress));
    assert(progress.result==Injection_Completed && !injection.active);
    assert(i2s_injection_claim(Z_AXIS,false)); // state_await_idle release
    unsigned old_writes=writes,old_modes=mode_changes,old_events=events;
    motion.id++;
    assert(!injection_submit(&motion));
    assert(!injection.active && writes==old_writes && mode_changes==old_modes && events==old_events);
    // Isolate the reason: the identical valid request passes when ownership returns.
    assert(i2s_injection_claim(Z_AXIS,true));
    assert(injection_submit(&motion));
    assert(injection.active && !locks);
    puts("Post-cut valid request: rejected without Z claim; accepted with claim. PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='thc-ownership-') as t:
    src=Path(t)/'test.c';exe=Path(t)/'test.exe';src.write_text(source)
    subprocess.run([a.cc,'-DSTEP_INJECT_STREAM=1','-I'+str(ROOT/'main'),str(src),
                    str(ROOT/'main/i2s_injection.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
