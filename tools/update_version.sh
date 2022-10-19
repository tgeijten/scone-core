#!/bin/sh

# Create a file repository_revision.h that defines a macro REPOSITORY_REVISION
echo -n '#define SCONE_REPOSITORY_REVISION ' > "./src/repository_revision.h"
git rev-list --count HEAD >> "./src/repository_revision.h"

# Create a file .version that contains the current commit count
git rev-list --count HEAD > "./.version"