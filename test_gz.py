import psutil
gz_running = False
for proc in psutil.process_iter(['name', 'cmdline']):
    try:
        name = proc.info.get('name', '') or ''
        cmdline = proc.info.get('cmdline', []) or []
        if 'ruby' in name or 'gz' in name or 'ign' in name:
            if any('gz' in arg or 'ign' in arg for arg in cmdline):
                gz_running = True
                print(f"Detected running: {name} {cmdline}")
    except Exception as e:
        pass

print("gz_running:", gz_running)
