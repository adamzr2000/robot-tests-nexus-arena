# Build one image per subdirectory, tagged "<name>:latest".
#
# To add a new image as the project grows: create dockerfiles/<name>/Dockerfile
# and append <name> to this list. That's it — `make <name>` and `make all` pick
# it up automatically.
IMAGES := \
	go2-driver \
	go2-domain-bridge \
	robot-trajectories \
	bags-collector \
	ros-humble-dev \
	roverx-tests

.PHONY: all clean $(IMAGES)

# Default target: builds every image listed above.
all: $(IMAGES)
	@echo ""
	@echo "========================================================"
	@echo "   ✅  ALL IMAGES BUILT SUCCESSFULLY"
	@echo "========================================================"

# Build a single image: `make go2-driver`
$(IMAGES):
	@echo ">> 📦 Building [$@:latest]..."
	@docker build -t $@:latest ./$@

# Remove the images built by this file
clean:
	@echo ">> 🧹 Removing project images..."
	-docker rmi $(addsuffix :latest,$(IMAGES))
