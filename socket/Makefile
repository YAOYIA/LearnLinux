CC:=gcc
CFLAGS:=-g -O0
BUILD_DIR:=build
TARGET := $(BUILD_DIR)/single_conn_server

# 默认目标
all: $(TARGET)

# num_endianess_convert:num_endianess_convert.c
# 	${CC} -o $@ $^
# 	./$@
# 	rm ./$@

# inet_endian_convert:inet_endian_convert.c
# 	${CC} -o $@ $^
# 	./$@
# 	rm ./$@

# ${TARGET}:single_conn_client.c
# 	@mkdir -p $(BUILD_DIR)   
# 	${CC} $(CFLAGS) -o $@ $^
# 	# ./$@
# 	# rm ./$@

${TARGET}:single_conn_server.c
	${CC} -o $@ $^
	# ./$@
	# rm ./$@

# without_buf_test:without_buf_test.c
# 	${CC} -o $@ $^
# 	./$@
# 	rm ./$@