CC:=gcc
BUILD_DIR:=build
CFLAGS:=-g -O0

TARGET:=${BUILD_DIR}/epoll_test
all: $(TARGET)


${TARGET}:epoll_test.c
	@mkdir -p $(BUILD_DIR)   
	${CC} ${CFLAGS} -o $@ $^
	# ${CC} -o $@ $^
