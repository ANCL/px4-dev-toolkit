# Geometric physical moment

The toolkit computes a physical Lee-style body moment with inertia, gyroscopic coupling, and angular acceleration feedforward. A separate physical wrench adapter converts it into normalized PX4 commands.

The adapter currently requires exactly four rotors, physical propulsion data, and matching PX4 allocator data. A vehicle YAML alone cannot add support for six rotors.
