"""tests/run_all.py - roda todas as funcoes test_* de tests/test_*.py (sem pytest).
Rode:  python -m tests.run_all"""
import glob
import importlib
import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    fails = n = 0
    for path in sorted(glob.glob(os.path.join(HERE, "test_*.py"))):
        mod = importlib.import_module("tests." + os.path.basename(path)[:-3])
        for name in sorted(dir(mod)):
            fn = getattr(mod, name)
            if name.startswith("test_") and callable(fn):
                n += 1
                try:
                    fn()
                except Exception:
                    fails += 1
                    print(f"FALHA {mod.__name__}.{name}")
                    traceback.print_exc()
    print(f"{n - fails}/{n} testes passaram")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
