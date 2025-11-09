# Spring Bones

Spring Bones is a Blender add-on that brings real-time spring dynamics to armatures so you can give bones a reactive, secondary motion without setting up complex simulations manually.

## Features
- Adds spring dynamics to single or multiple bones for lively rigs.
- Provides viewport controls for tuning stiffness, damping, and visualization.
- Includes handlers that keep simulations responsive while you animate.

## Blender Remote Repository (Extensions)

Add this URL in Blender 4.x:
Edit → Preferences → Get Extensions → Repositories → + Add Remote Repository

    https://sage-bomb.github.io/blender_spring_armature/index.json

Then you can install/update the extension directly from Blender.

### Local build & test

Build a distributable `.zip` of the extension:

```bash
blender --command extension build --source-dir spring_bones --output-dir ./dist
```

Generate or refresh the repository `index.json`:

```bash
blender --command extension server-generate --repo-dir ./dist
```

The commands above require Blender 4.4 or newer, matching the extension's compatibility.
