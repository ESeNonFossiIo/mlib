import sys
import math

def compare_files(expected_file, actual_file, tolerance=1e-6):
    try:
        with open(expected_file, 'r') as f1, open(actual_file, 'r') as f2:
            expected_lines = f1.readlines()
            actual_lines = f2.readlines()
    except FileNotFoundError as e:
        print(f"Error opening files: {e}")
        return False

    if len(expected_lines) != len(actual_lines):
        print(f"Line count mismatch: Expected {len(expected_lines)}, got {len(actual_lines)}")
        print("-- Expected --")
        for line in expected_lines[-10:]:
            print(line)
        print("-- Actual --")
        for line in actual_lines[-10:]:
            print(line)
        return False

    for line_num, (line1, line2) in enumerate(zip(expected_lines, actual_lines), 1):
        # .split() automatically handles spaces, tabs, and Windows/Linux line endings
        tokens1 = line1.split()
        tokens2 = line2.split()

        if len(tokens1) != len(tokens2):
            print(f"Token mismatch on line {line_num}.")
            return False

        for col_num, (t1, t2) in enumerate(zip(tokens1, tokens2), 1):
            try:
                # Try to compare them as floating point numbers
                val1 = float(t1)
                val2 = float(t2)
                
                # math.isclose handles absolute and relative tolerance perfectly
                if not math.isclose(val1, val2, rel_tol=tolerance, abs_tol=tolerance):
                    print(f"Math difference at line {line_num}, col {col_num}: Expected {val1}, got {val2}")
                    return False
            except ValueError:
                # If they aren't numbers (like "NaN" or standard text), compare as strings
                if t1 != t2:
                    print(f"Text difference at line {line_num}, col {col_num}: Expected '{t1}', got '{t2}'")
                    return False
                    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script_diff.py <expected_file> <actual_file> [tolerance]")
        sys.exit(1)

    expected = sys.argv[1]
    actual = sys.argv[2]
    tol = float(sys.argv[3]) if len(sys.argv) > 3 else 1e-6

    # Exit with 0 if they match, 1 if they fail (CMake relies on these exit codes)
    if compare_files(expected, actual, tol):
        sys.exit(0)
    else:
        sys.exit(1)
