# fuzzy membership functions
import matplotlib.pyplot as plt

num_inputs = 50
x_values = [i for i in range(num_inputs)]

# fuzzy membership dingin, sedang, panas
def create_membership(n_set, range_min, range_max):
    # Normalize the range
    member_values = [] # get value points

    for i in range(n_set):
        member_values.append(range_min + (range_max - range_min) * i / (n_set - 1))
        print(member_values[i])
    
    return member_values

def create_plot(member_values):

    fuzzy_membership = [[0 for _ in range(num_inputs)] for _ in range(len(member_values))]

    # Create the fuzzy membership function
    for i in range(len(member_values)):
      for j in range(len(x_values)):
        x = x_values[j]

        if i == 0:
            a = member_values[i]
            b = member_values[i+1]
            if x < a:
                fuzzy_membership[i][j] = 1
            elif a <= x <= b:
                fuzzy_membership[i][j] = (b - x) / (b - a)
        
        elif i == len(member_values) - 1:
            a = member_values[i-1]
            b = member_values[i]
            if x > b:
                fuzzy_membership[i][j] = 1
            elif a <= x <= b:
                fuzzy_membership[i][j] = (x - a) / (b - a)

        else:
            a = member_values[i-1]
            b = member_values[i]
            c = member_values[i+1]

            if a <= x <= b:
                fuzzy_membership[i][j] = (x - a) / (b - a)
            elif b < x <= c:
                fuzzy_membership[i][j] = (c - x) / (c - b)

    # print(fuzzy_membership)

    # Plot the fuzzy membership function
    plt.figure(figsize=(10, 6))
    for i in range(len(fuzzy_membership)):
        plt.plot(x_values, fuzzy_membership[i], label=f'Set {i+1}')
    plt.xlabel("Temperature")
    plt.ylabel("Membership degree")
    plt.title("Fuzzy Membership Functions")
    plt.xlim(0, 50)
    plt.ylim(-0.25, 1.25)
    plt.legend()
    plt.grid(True)
    plt.show()

    return fuzzy_membership

hot_encoding = create_membership(3, 10, 40)
fuzzy_membership = create_plot(hot_encoding)

value = 30

# Check the membership degree for each set
for i in range(len(fuzzy_membership)):
    membership_degree = fuzzy_membership[i][value]
    print(f"Membership degree for Set {i+1} at value {value}: {membership_degree}")



        

