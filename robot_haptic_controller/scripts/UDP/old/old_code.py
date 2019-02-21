#old code





# joint_vector = []
# for joint in range(0, 27):
#   joint_vector.append(struct.unpack("f", data[4 + 4 * joint:4 + 4 * joint + 4])[0])

# print(joint_vector)


# old code
# joint_vector.append(struct.unpack("f", byte_array[4 + 4 * joint:4 + 4 * joint + 4])[0])

# byte_array = bytearray(data)


# print(type(data))

# print("sys.getsizeof(data):")
# print("----\ndata:\n" + str(data))
# print(sys.getsizeof(data))

# print ':'.join(x.encode('hex') for x in data)
# print(len(data))

# msg_length = data[0].encode('hex')
# print(type(data[0]))
# print("msg_length (as hex):" + str(msg_length))


# print("====")
# print(struct.unpack("c", data[0]))
# print(data[0].encode('hex'))
# print(struct.unpack("s", data[0]))


# print(type(byte_array))
# print("byte-array:")
# print("msg length: " + str(byte_array[0]))
# print("first value: " + str(byte_array[4:4 + 4]))
# print("first value: " + str(byte_array[4:4+4]))
# print(struct.unpack("f", byte_array[4:4 + 4]))