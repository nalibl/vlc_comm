import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import time
from time import sleep
import numpy as n

    
# Constants
AI_PIN = "P9_39" # analog in
DO_PIN = "P9_12" # gpio
PWM_PIN = "P9_14" # pwm
BUFFER_SIZE = 64


def system_timing(cali_iterations=100)
    timing_delay = np.zeros(cali_iterations)
    adc_delay = np.zeros(cali_iterations)
    for idx in range(cali_iterations):
        t1 = time.time()
        timing_delay[idx] = time.time()-t1
        t1 = time.time()
        adc_delay[idx] = time.time()-t1
    timing_delay = np.mean(timing_delay)
    adc_delay = np.mean(adc_delay)
    return timing_delay, adc_delay
    
    
def set_signal_params(clock)
    sample_per = np.ceil(eff_clock)
    low_per = 34 * eff_clock
    high_per = 2 * low_per
    carrier_per = (low_per + high_per)/2
    symbol_len = low_per * 4
    return sample_per, low_per, high_per, carrier_per, symbol_len

def freq_demod(signal, kernel):
    """
    Derivative, absolute value, threshold, convolve, outliers, threshold
    """
    sig_deriv = signal[:-1] - signal[1:]
    sig_deriv_abs = np.abs(sig_deriv)
    sig_deriv_abs_thresh = sig_deriv_abs > np.mean(sig_deriv_abs)
    freq_vec = np.convolve(sig_deriv_abs_thresh, kernel)
    min_val = kernel.size * 2  # add tolerance
    max_val = max_val * 2
    freq_vec_outlier = np.max(np.min(freq_vec, max_val), min_val)
    bin_vec = freq_vec_outlier > (min_val+max_val)/2
    return bin_vec

# Pin setup
ADC.setup()
GPIO.setup(DO_PIN, GPIO.OUT)

print('started')

pwm_freq = 50
freq_div_factor = 10
n_samples = 500
PWM.start(PWM_PIN, 50, frequency=pwm_freq)
read_buffer = np.zeros(n_samples)

time.sleep(1)

for i in range(n_samples):
    read_buffer[i] = ADC.read(AI_PIN)
    time.sleep(1/(pwm_freq*freq_div_factor))
#     PWM.set_frequency(PWM_PIN, 2 ** (i+2))

print(read_buffer)

print('ended')

PWM.stop(PWM_PIN)
PWM.cleanup()
GPIO.cleanup()

# Main loop
# while True:
    # PWM.start(DO_PIN, 50, frequency=1)