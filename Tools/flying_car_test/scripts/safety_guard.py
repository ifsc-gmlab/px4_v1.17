#!/usr/bin/env python3

try:
    from flying_car_test.safety import main
except ImportError as error:
    raise ImportError("flying_car_test.safety.main is unavailable") from error


if __name__ == "__main__":
    main()
