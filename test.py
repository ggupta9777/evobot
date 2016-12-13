def test(chromosome, len_sub):
	par = []
	for i in range(len(chromosome)/len_sub):
		temp = chromosome[i*len_sub:(i+1)*len_sub]
		num = int(''.join(map(str,temp)))
		num = float(int(str(num),2))
		par.append(num/100-5)
	return par

print test([1]*20,10)