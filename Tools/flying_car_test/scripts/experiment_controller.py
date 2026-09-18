#!/usr/bin/env python3

try:
    from flying_car_test.controller import main
except ImportError as error:
    raise ImportError("flying_car_test.controller.main is unavailable") from error


if __name__ == "__main__":
    main()
