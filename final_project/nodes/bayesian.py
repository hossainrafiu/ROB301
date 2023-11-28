import matplotlib.pyplot as plt

class State:
    measProb = 1/11
    controlProb = 0
    measAccuracy = 0
    colour = 'c'

states = [State() for i in range(11)]


# room 2 is at index 0, yellow:
states[0].colour = 'y'
# room 3 is at index 1, green:
states[1].colour = 'g'
# room 4 is at index 2, blue:
states[2].colour =  'b'
# room 5 is at index 3, orange:
states[3].colour = 'o'
# room 6 is at index 4, orange:
states[4].colour =  'o'
# room 7 is at index 5, green:
states[5].colour =  'g'
# room 8 is at index 6, blue:
states[6].colour = 'b'
# room 9 is at index 7, orange:
states[7].colour = 'o'
# room 10 is at index 8, yellow:
states[8].colour = 'y'
# room 11 is at index 9, green:
states[9].colour = 'g'
# room 12 is at index 10, blue:
states[10].colour = 'b'

stateModel = [[0.85, 0.05, 0.05],
              [0.1,0.9,0.1]]

measColours = ['b','g','y','o','']
measModel = [[0.6,0.2,0.05,0.05],
             [0.2,0.6,0.05,0.05],
             [0.05,0.05,0.65,0.2],
             [0.05,0.05,0.15,0.6],
             [0.1,0.1,0.1,0.1]]

prob = []
for state in states:
    prob.append(state.measProb)

# plt.bar(range(2,13),prob)
# plt.show()

def update(u_k, z_k):
    # state prediction:
    probs = []
    for x_k in range(len(states)):
        prob = 0
        # recompute state control probabilities:
        for state in states:
            state.controlProb = 0
        
        if u_k == -1:
            states[x_k - 1].controlProb = 0.05
            states[x_k].controlProb = 0.1
            if x_k != 10:
                states[x_k + 1].controlProb = 0.85
            else:
                states[0].controlProb = 0.85
        
        elif u_k == 0:
            states[x_k - 1].controlProb = 0.05
            states[x_k].controlProb = 0.90
            if x_k != 10:
                states[x_k + 1].controlProb = 0.05
            else:
                states[0].controlProb = 0.05
        
        elif u_k == 1:
            states[x_k - 1].controlProb = 0.85
            states[x_k].controlProb = 0.1
            if x_k != 10:
                states[x_k + 1].controlProb = 0.05
            else:
                states[0].controlProb = 0.05
        
        for state in states:
            prob += state.controlProb * state.measProb
        
        probs.append(prob)


    for x_k in range(len(states)):
        states[x_k].measProb = probs[x_k]

    # plt.bar(range(2,13),probs)
    # plt.show()

    # calculate probability of getting our given measurement: z_k+1 given the state:
    for curr_state in states:
        measIndex = measColours.index(curr_state.colour)
        curr_state.measAccuracy = measModel[measColours.index(z_k)][measIndex]

    # state update:
    total = 0
    for curr_state in states:
        curr_state.measProb = (curr_state.measProb)*(curr_state.measAccuracy)
        total += curr_state.measProb # for normalizing later

    most_likely_state = 0
    greatestProb = 0
    # normalize all:
    for curr_state in states:
        curr_state.measProb = curr_state.measProb / total
        if curr_state.measProb > greatestProb:
            most_likely_state = curr_state
            greatestProb = curr_state.measProb

    x_k = most_likely_state

    prob = []
    for state in states:
        prob.append(state.measProb)
    # print(sum(prob))

    plt.bar(range(2,13),prob)
    plt.show()

update(+1, 'o')
update(+1, 'y')
update(+1, 'g')
update(+1, 'b')
update(+1, '')
update(+1, 'g')
update(+1, 'b')
update(0, 'g')
update(+1, 'o')
update(+1, 'y')
update(+1, 'g')
update(+1, 'b')