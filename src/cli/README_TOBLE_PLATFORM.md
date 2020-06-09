OpenThread CLI - ToBLE Platform Example

The OpenThread ToBLE Platform APIs will be invoked via the OpenThread CLI.

# Build
```
make -f ./examples/Makefile-simulation TOBLE=1
```

# Command Details

## adv start <interval> <advData>

## adv stop

## scan start <interval> <window>

## connect start <addrType> <address>

## connect stop

## send <connId> <length>

```
toble adv start 200 123456
toble scan stop
toble connect start 1 000000000002
toble send 0 20
```
