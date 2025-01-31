# Assembly

Abstraction of vehicle configurations and combinations. 

We expect a small handful of potential combinations (~12), and we don't expect arbitrary combinations.
Therefor, we can greatly simplify the configuration process by explicitly supporting the known combinations.

See [Header](include/assembly.h) for description of the interface.

Ready to use:
- [Tractor + Trailer](include/tractor-trailer.h)
- [Tractor + Revoy + Trailer](include/tractor-revoy-trailer.h)

TODO:
- Tractor
- Tractor + Shorty
- Tractor + Shorty + Shorty
- Tractor + Revoy
- Tractor + Revoy + Shorty
- Tractor + Revoy + Shorty + Shorty
- Revoy
- Revoy + Trailer
- Revoy + Shorty
- Revoy + Shorty + Shorty

