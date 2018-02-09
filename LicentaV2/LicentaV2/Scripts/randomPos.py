import random


def RandomPosition():
	mins = [-5, 7, -5]
	maxes = [5, 15, 5]

	return (random.uniform(mins[0], maxes[0]), random.uniform(mins[1], maxes[1]), random.uniform(mins[2], maxes[2]))