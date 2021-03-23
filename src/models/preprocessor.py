import os, sys, zipfile

'''
for subdir, dirs, files in os.walk('.'):
	# print(subdir)
	# print('')
	for file in files:
		if file[-4:] == '.zip':
			
			if subdir[-6:] != 'meshes':
				with zipfile.ZipFile(subdir+'/'+file, 'r') as zipobj:
					zipobj.extractall(subdir+'/meshes/')
				daes = os.listdir(subdir+'/meshes/')
				for dae in daes:
					if dae[-3:] == 'obj':
						os.rename(subdir+'/meshes/'+dae, subdir+'/meshes/object.obj')
					if dae[-3:] == 'mtl':
						os.rename(subdir+'/meshes/'+dae, subdir+'/meshes/material.mtl')
			# print(subdir, file)

for subdir, dirs, files in os.walk('.'):
	for file in files:
		if file == 'object.obj':
			print(subdir+'/object.obj')
			with open(subdir+'/object.obj', 'r') as f:
				data = f.readlines()
			data[3] = 'mtllib material.mtl\n'
			with open(subdir+'/object.obj', 'w') as f:
				f.writelines(data)
'''
for subdir, dirs, files in os.walk('.'):
	for file in files:
		if file == 'model.sdf':
			try:
				with open(subdir+'/model.sdf', 'r') as f:
					data = f.readlines()
				for i,line in enumerate(data):
					res = line
					line = line.replace(" ","")
					line = line.replace("\t","")
					# print(line)

					if line[:5] == '<uri>':
						line = line.split('/')
						line[4] = 'object.obj<'
						res = '/'.join(line)
						data[i] = res 
				with open(subdir+'/model.sdf', 'w') as f:
					f.writelines(data)
			except:
				print('wrong format', subdir+'/model.sdf')