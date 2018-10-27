import cloudpickle as pickle
import os.path
import os
import errno
import csv




def save_data(folder, name, data):
    model_file = folder + name + '.pkl'

    if not os.path.exists(os.path.dirname(model_file)):
        try:
            os.makedirs(os.path.dirname(model_file))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    with open(model_file, 'w+') as f:
        pickle.dump(data, f)

def save_csv_data(folder, name, data):
    with open(folder + name + '.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(np.size(data, 0) - 10):
            wr.writerow(data[i])

def load_data(folder, name):
    model_file = folder + name + '.pkl'
    if os.path.exists(model_file):
        data = pickle.load(open(model_file, 'rb'))
        return data
    else:
        raise Exception("No {} can be found!".format(model_file))
