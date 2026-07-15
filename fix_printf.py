"""Fix printf in main.c by line number."""
with open('stm32_uwb/Core/Src/main.c', 'r', encoding='utf-8', newline='') as f:
    lines = f.readlines()

# Find lines containing the broken format string
for i, line in enumerate(lines):
    if 'GPS: %ld.%07ld' in line:
        # Fix: replace the broken \r\n escape (which became real newlines) with proper escapes
        # Line i has the format string ending abruptly
        # Line i+1 is blank or just whitespace
        # Line i+2 starts with '",'

        # Rebuild the printf as clean lines
        indent1 = '\t\t\t  printf("[%lu] x=%d.%02d y=%d.%02d z=%d.%02d  "\n'
        indent2 = '\t\t\t\t\t "vx=%d.%02d vy=%d.%02d vz=%d.%02d  "\n'
        indent3 = '\t\t\t\t\t "GPS: %ld.%07ld,%ld.%07ld,%ld.%03ldm\\r\\n",\n'

        # Find the start of the printf (search backwards from i)
        printf_start = i
        while printf_start > 0:
            if 'printf(' in lines[printf_start] or 'printf' in lines[printf_start]:
                if lines[printf_start].strip().startswith('printf('):
                    break
            printf_start -= 1

        # Find where the printf ends (the '));' line)
        printf_end = i
        while printf_end < len(lines):
            if 'labs(alt_mm' in lines[printf_end]:
                printf_end += 1
                break
            printf_end += 1

        print(f'Replacing lines {printf_start+1}-{printf_end}')

        new_block = [
            indent1,
            indent2,
            indent3,
        ]
        # Copy the remaining printf args (skip the broken lines)
        # Find args start: the line with '(unsigned long)'
        for j in range(printf_start, printf_end):
            if 'unsigned long' in lines[j]:
                new_block.extend(lines[j:printf_end])
                break

        lines[printf_start:printf_end] = new_block
        break

with open('stm32_uwb/Core/Src/main.c', 'w', encoding='utf-8', newline='') as f:
    f.writelines(lines)
print('Done')
