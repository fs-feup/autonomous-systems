import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.express as px
import pandas as pd
import os
import sys
import shutil
import atexit

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "cloud_storage"))
)
from bucket_operations import list_blobs, download_csv_from_bucket_to_folder

# Initialize the Dash app
app = dash.Dash(__name__)

# List of available CSV files in the bucket
csv_files = list_blobs("as_evaluation")

# Define available dashboards
available_dashboards = [
    "Evaluator_perception",
    "Evaluator_state_estimation",
    "Evaluator_planning",
    "Evaluator_control",
    "Perception",
    "State_estimation",
    "Planning",
]

# Define the main layout of the app
app.layout = html.Div(
    [
        dcc.Store(id="stored-csv-data", storage_type="memory"),
        html.H1("Select Dashboard"),
        dcc.Dropdown(
            id="dashboard-dropdown",
            options=[
                {"label": dashboard, "value": dashboard}
                for dashboard in available_dashboards
            ],
            value=None,
            placeholder="Select a dashboard",
        ),
        html.Div(id="dashboard-content"),
    ]
)


# Define the layout for each dashboard
def get_dashboard_layout(dashboard):
    dashboard_conditions = {
        "Evaluator_perception": "evaluator/perception",
        "Evaluator_state_estimation": "evaluator/se",
        "Evaluator_planning": "evaluator/planning",
        "Evaluator_control": "evaluator/control",
        "Perception": "perception",
        "State_estimation": "state_est",
        "Planning": "planning",
    }

    condition = dashboard_conditions.get(dashboard, "")

    # Filter CSV files based on condition
    csv_files = [file for file in list_blobs("as_evaluation") if condition in file]

    return html.Div(
        [
            html.H2(f"{dashboard} CSV files"),
            dcc.Dropdown(
                id=f"csv-dropdown-{dashboard}",
                options=[{"label": f, "value": f} for f in csv_files],
                value=[],
                multi=True,
                placeholder="Select CSV files",
            ),
            html.Div(
                [
                    dcc.Graph(
                        id=f"graph1-{dashboard}",
                        style={"width": "70%", "height": "500px"},
                    ),
                    html.Div(
                        [
                            html.Label("Y-Axis"),
                            dcc.Dropdown(
                                id=f"graph1-{dashboard}-metrics-dropdown",
                                multi=True,
                                style={"minWidth": "200px"},
                            ),
                            html.Br(),
                            html.Label("X-Axis"),
                            dcc.Dropdown(
                                id=f"x-axis-dropdown-{dashboard}-1",
                                style={"minWidth": "200px"},
                            ),
                        ],
                        id=f"graph1-metrics-{dashboard}",
                        style={"minWidth": "200px", "width": "25%"},
                    ),
                ],
                style={"display": "flex"},
            ),
            html.Div(
                [
                    dcc.Graph(
                        id=f"graph2-{dashboard}",
                        style={"width": "70%", "height": "500px"},
                    ),
                    html.Div(
                        [
                            html.Label("Y-Axis"),
                            dcc.Dropdown(
                                id=f"graph2-{dashboard}-metrics-dropdown",
                                multi=True,
                                style={"minWidth": "200px"},
                            ),
                            html.Br(),
                            html.Label("X-Axis"),
                            dcc.Dropdown(
                                id=f"x-axis-dropdown-{dashboard}-2",
                                style={"minWidth": "200px"},
                            ),
                        ],
                        id=f"graph2-metrics-{dashboard}",
                        style={"minWidth": "200px", "width": "25%"},
                    ),
                ],
                style={"display": "flex"},
            ),
            html.Div(
                [
                    dcc.Graph(
                        id=f"graph3-{dashboard}",
                        style={"width": "70%", "height": "500px"},
                    ),
                    html.Div(
                        [
                            html.Label("Y-Axis"),
                            dcc.Dropdown(
                                id=f"graph3-{dashboard}-metrics-dropdown",
                                multi=True,
                                style={"minWidth": "200px"},
                            ),
                            html.Br(),
                            html.Label("X-Axis"),
                            dcc.Dropdown(
                                id=f"x-axis-dropdown-{dashboard}-3",
                                style={"minWidth": "200px"},
                            ),
                        ],
                        id=f"graph3-metrics-{dashboard}",
                        style={"minWidth": "200px", "width": "25%"},
                    ),
                ],
                style={"display": "flex"},
            ),
        ]
    )


# Callback to update the layout based on selected dashboard
@app.callback(
    Output("dashboard-content", "children"), Input("dashboard-dropdown", "value")
)
def update_dashboard(selected_dashboard):
    """!
    Update the layout based on the selected dashboard.

    Args:
        selected_dashboard: The selected dashboard.
    """
    if selected_dashboard:
        return get_dashboard_layout(selected_dashboard)
    return html.Div()


def download_and_combine_csvs(selected_csvs, temp_folder):
    """!
    Download and combine the selected CSV files.

    Args:
        selected_csvs: The selected CSV files.
        temp_folder: The temporary folder to store the CSV files.
    """
    combined_df = pd.DataFrame()
    for csv in selected_csvs:
        download_csv_from_bucket_to_folder("as_evaluation", csv, temp_folder, csv)
        temp_df = pd.read_csv(os.path.join(temp_folder, csv))
        temp_df["Source"] = csv
        combined_df = pd.concat([combined_df, temp_df], ignore_index=True)
    return combined_df


def create_update_metric_dropdowns_callback(dashboard, graph_number):
    @app.callback(
        [
            Output(f"graph{graph_number}-{dashboard}-metrics-dropdown", "options"),
            Output(f"x-axis-dropdown-{dashboard}-{graph_number}", "options"),
        ],
        [
            Input(f"csv-dropdown-{dashboard}", "value"),
            Input(f"stored-csv-data", "data"),
        ],
        State(f"csv-dropdown-{dashboard}", "value"),
    )
    def update_metric_dropdowns(selected_csvs, stored_data, current_selected_csvs):
        """!
        Update the metric dropdowns based on the selected CSV files.

        Args:
            selected_csvs: The selected CSV files.
            stored_data: The stored CSV data.
            current_selected_csvs: The currently selected CSV files.
        """
        if not selected_csvs:
            return [], []

        temp_folder = "src/cloud_storage/temp"
        if stored_data is None or set(current_selected_csvs) != set(selected_csvs):
            combined_df = download_and_combine_csvs(selected_csvs, temp_folder)
        else:
            combined_df = pd.read_json(stored_data, orient="split")

        columns = list(combined_df.columns)
        options = [{"label": col, "value": col} for col in columns]

        return options, options


def create_update_graph_callback(graph_id, dashboard, graph_number, graph_type="line"):
    @app.callback(
        Output(graph_id, "figure"),
        [
            Input(f"csv-dropdown-{dashboard}", "value"),
            Input(f"graph{graph_number}-{dashboard}-metrics-dropdown", "value"),
            Input(f"x-axis-dropdown-{dashboard}-{graph_number}", "value"),
            Input(f"stored-csv-data", "data"),
        ],
        State(f"csv-dropdown-{dashboard}", "value"),
    )
    def update_graph(
        selected_csvs, metrics, x_axis, stored_data, current_selected_csvs
    ):
        """!
        Update the graph based on the selected CSV files and metrics.
        Args:
            selected_csvs: The selected CSV files.
            metrics: The selected metrics.
            x_axis: The selected x-axis.
            stored_data: The stored CSV data.
            current_selected_csvs: The currently selected CSV files.
        """
        if not selected_csvs or not metrics:
            return {}

        temp_folder = "src/cloud_storage/temp"
        if stored_data is None or set(current_selected_csvs) != set(selected_csvs):
            combined_df = download_and_combine_csvs(selected_csvs, temp_folder)
        else:
            combined_df = pd.read_json(stored_data, orient="split")

        if x_axis not in combined_df.columns:
            x_axis = "time"
            if "time" not in combined_df.columns:
                combined_df["time"] = range(len(combined_df))

        df_melted = combined_df.melt(id_vars=[x_axis, "Source"], value_vars=metrics)
        df_melted["Source_Metric"] = df_melted["Source"] + " - " + df_melted["variable"]
        df_melted = df_melted[df_melted["variable"].isin(metrics)]

        if graph_type == "line":
            fig = px.line(
                df_melted,
                x=x_axis,
                y="value",
                color="Source_Metric",
                labels={"value": "Metrics", x_axis: x_axis},
                title="Line Graph",
            )
        elif graph_type == "scatter":
            fig = px.scatter(
                df_melted,
                x=x_axis,
                y="value",
                color="Source_Metric",
                labels={"value": "Metrics", x_axis: x_axis},
                title="Scatter Plot",
            )

        return fig


# Register callbacks for all dashboards and graphs
for dashboard in available_dashboards:
    create_update_metric_dropdowns_callback(dashboard, 1)
    create_update_metric_dropdowns_callback(dashboard, 2)
    create_update_metric_dropdowns_callback(dashboard, 3)
    create_update_graph_callback(f"graph1-{dashboard}", dashboard, 1, "line")
    create_update_graph_callback(f"graph2-{dashboard}", dashboard, 2, "line")
    create_update_graph_callback(f"graph3-{dashboard}", dashboard, 3, "scatter")


# Cleanup function to remove the temp folder
def cleanup_temp_folder():
    temp_folder = "src/cloud_storage/temp"
    if os.path.exists(temp_folder):
        shutil.rmtree(temp_folder)


atexit.register(cleanup_temp_folder)

# Run the Dash app
if __name__ == "__main__":
    app.run_server(debug=True)
