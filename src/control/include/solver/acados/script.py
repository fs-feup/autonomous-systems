def main():
    previous_input = None
    iteration = 1
    
    while True:
        print(f"Iteration {iteration}: Enter text (or 'quit' to exit). Press Ctrl+D (or Ctrl+Z on Windows) when done:")
        lines = []
        try:
            while True:
                line = input()
                lines.append(line)
        except EOFError:
            pass
        
        current_input = "\n".join(lines)
        
        if current_input.lower() == 'quit':
            break
        
        if previous_input is None:
            print("First input received. Enter next input to compare.\n")
        else:
            if current_input == previous_input:
                print("✓ Input is identical to previous input.\n")
            else:
                print("✗ Input differs from previous input:")
                print_difference(previous_input, current_input)
                print()
        
        previous_input = current_input
        iteration += 1


def print_difference(prev, curr):
    max_len = max(len(prev), len(curr))
    
    for i in range(max_len):
        prev_char = prev[i] if i < len(prev) else "∅"
        curr_char = curr[i] if i < len(curr) else "∅"
        
        if prev_char != curr_char:
            print(f"  Position {i}: '{prev_char}' → '{curr_char}'")


if __name__ == "__main__":
    main()