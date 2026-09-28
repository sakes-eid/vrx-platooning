def get_stage_parameters(
    config,
    controller,
    stage,
):

    controller_config = (
        config[
            controller
        ]
    )

    stage_config = (
        controller_config[
            stage
        ]
    )

    return stage_config.get(
        'parameters',
        {}
    )


def validate_search_space(
    parameters,
):

    errors = []

    for name, bounds in parameters.items():

        if (
            'min'
            not in bounds
            or
            'max'
            not in bounds
        ):

            errors.append(
                f'{name}: missing min/max'
            )

            continue

        minimum = float(
            bounds[
                'min'
            ]
        )

        maximum = float(
            bounds[
                'max'
            ]
        )

        if minimum > maximum:

            errors.append(
                f'{name}: min > max'
            )

    return errors
