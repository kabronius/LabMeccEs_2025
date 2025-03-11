import matplotlib.pyplot as plt

file = open('data.txt','r')
data_dict = {}
first_line = file.readline()                 # <-- 'PH;ORP;DO;TEMP\n'
keys = first_line[:-1].split(';')            # <-- ['PH','ORP','DO','TEMP']
for k in keys:
	data_dict[k] = []                         # <-- {'PH':[],'ORP':[],'DO':[],'TEMP':[]}
for line in file:
	data = line[:-1].split(';')              # <-- ['ww.w','xx.x','yy.y','zz.z']
	for k in keys:
		data_dict[k].append(float(data.pop(0)))
file.close()
print('### DATA:')
for k in keys:
	print(k+':\t'+str(data_dict[k]))
print('\n### MEAN VALUES:')
for k in keys:
	mean = sum(data_dict[k])/len(data_dict[k])
	print(k+':\t'+str(round(mean,2)))

# Plot a figure and stop at each loop cycle
for k in keys:
    plt.figure()
    plt.plot(data_dict[k])
    plt.ylabel(k)
    plt.show()

# Subplot
plt.figure()
plt.subplot(221)
plt.plot(data_dict['PH'],'ro')
plt.ylabel('PH')
plt.subplot(222)
plt.plot(data_dict['ORP'],'bo')
plt.ylabel('ORP')
plt.subplot(223)
plt.plot(data_dict['DO'],'go')
plt.ylabel('DO')
plt.subplot(224)
plt.plot(data_dict['TEMP'],'yo')
plt.ylabel('TEMP')

plt.show()