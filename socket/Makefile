CC:=gcc
CFLAGS:=-g -O0
BUILD_DIR:=build
TARGET := $(BUILD_DIR)/udp_server

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

# ${TARGET}:single_conn_server.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@

# without_buf_test:without_buf_test.c
# 	${CC} -o $@ $^
# 	./$@
# 	rm ./$@


# ${TARGET}:multi_conn_threads_server.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@

# ${TARGET}:multi_conn_threads_client.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@

# ${TARGET}:multi_conn_processes_server.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@

# ${TARGET}:multi_conn_processes_client.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@

# ${TARGET}:udp_client.c
# 	${CC} -o $@ $^
# 	# ./$@
# 	# rm ./$@
${TARGET}:udp_server.c
	${CC} -o $@ $^
	# ./$@
	# rm ./$@
