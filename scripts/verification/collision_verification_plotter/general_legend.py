
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

COLORS = ["blue","red","green","yellow","purple"]
COLOR_MAP = {"blue":[0,0,255],"red":[255,0,0],"green":[0,127,0],"yellow":[255/255,165/255,0],"purple":[127,0,127]}

matplotlib.rc('font', **font)

if __name__ == "__main__":
    
    fig = plt.figure()

    legend_elements = [mlines.Line2D([0], [0], color=COLORS[0], lw=2, label='1 Future Step'),
                       mlines.Line2D([0], [0], color=COLORS[1], lw=2, label='2 Future Steps'),
                       mlines.Line2D([0], [0], color=COLORS[2], lw=2, label='3 Future Steps'),
                       mlines.Line2D([0], [0], color=COLOR_MAP[COLORS[3]], lw=2, label='4 Future Steps'),
                       mlines.Line2D([0], [0], color=COLORS[4], lw=2, label='5 Future Steps'),
                #        mlines.Line2D([0], [0], color=(0,0,0), lw=2, linestyle="dashed", label="Point of Collision"),
                       mlines.Line2D([0], [0], color=(0,0,0), marker='s', markersize=12, lw=0, label='Ego Vehicle',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0)),
                       mlines.Line2D([0], [0], color=(0,0,0), marker='o', markersize=12, lw=0, label='Other Vehicle',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0))
                       ]
    
    fig.legend(handles=legend_elements,ncol=len(legend_elements))
    fig.tight_layout()
    plt.gca().set_axis_off()
    
    
    # fig.canvas.draw()
    plt.show(block=True)
