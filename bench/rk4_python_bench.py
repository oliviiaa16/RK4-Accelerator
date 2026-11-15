import time
import csv

def f(x, y):
    return -y

def rk4_solve(y0, x0, h, steps):
    y = y0
    x = x0
    for _ in range(steps):
        k1 = f(x, y)
        k2 = f(x + h/2, y + h*k1/2)
        k3 = f(x + h/2, y + h*k2/2)
        k4 = f(x + h, y + h*k3)
        y += (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        x += h
    return y

def main():
    inputs = []
    with open('benchmark_inputs.csv', 'r') as f_in:
        reader = csv.DictReader(f_in)
        for row in reader:
            inputs.append((float(row['x']), float(row['y'])))
    
    start = time.perf_counter()
    results = []
    
    for x0, y0 in inputs:
        res = rk4_solve(y0, x0, 0.01, 100000)
        results.append(res)
        
    end = time.perf_counter()
    
    print("--- Python Standardized Benchmark ---")
    print(f"Processed {len(inputs)} equations sequentially.")
    print(f"Total Runtime: {(end - start) * 1000:.2f} ms")

if __name__ == '__main__':
    main()
