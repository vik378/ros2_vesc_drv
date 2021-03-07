import re

RE_RW_LINE = re.compile(r'(read|write)\(29, (".*?")(?:[.]+)?[,].*?')

if __name__ == "__main__":
    import sys

    sys.stdout.write("STRACE = [\n")
    for line in sys.stdin:
        res = RE_RW_LINE.findall(line)
        if len(res) > 0:
            sys.stdout.write(f"    b{res[0][1]},  # {res[0][0]}\n")
    sys.stdout.write("]\n")
