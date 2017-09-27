import scipy.misc as misc
import tensorflow as tf
import numpy as np
import json
import os


# TODO: Not a good place for these variables, move elsewhere.
_conv_stride = np.array([2,1,1,1,1])
_filtergroup_yn = np.array([0,1,0,1,1], dtype=bool)
_bnorm_yn = np.array([1,1,1,1,0], dtype=bool)
_relu_yn = np.array([1,1,1,1,0], dtype=bool)
_pool_stride = np.array([2,1,0,0,0]) # 0 means no pool
_pool_sz = 3
_bnorm_adjust = True

class SiamFC():
    """Deep-learning based one-shot object detection as described in:

    Title: Fully-Convolutional Siamese Networks for Object Tracking
    Authors: Bertinetto, Luca and Valmadre, Jack and Henriques, Jo{\~a}o F and Vedaldi, Andrea and
             Torr, Philip H S
    Book Title: ECCV 2016 Workshops
    Pages: 850 - 865
    Year: 2016

    Attributes:
        top_left (int, int): The x, y coordinates of the top left corner of the bounding box of the
        most recent image.
        bottom_right (int, int): The x, y coordinates of the bottom right corner of the bounding
        box of the most recent image.
    """


    def __init__(self):
        """Initialize siamese network for object detection"""

        self.pos_x_ph = tf.placeholder(tf.float64)
        self.pos_y_ph = tf.placeholder(tf.float64)
        self.z_sz_ph = tf.placeholder(tf.float64)
        self.x_sz0_ph = tf.placeholder(tf.float64)
        self.x_sz1_ph = tf.placeholder(tf.float64)
        self.x_sz2_ph = tf.placeholder(tf.float64)

        self._run_params = self._load_json_file('run.json')
        self._design_params = self._load_json_file('design.json')
        self._environment_params = self._load_json_file('environment.json')
        self._evaluation_params = self._load_json_file('evaluation.json')
        self._hyper_params = self._load_json_file('hyperparms.json')
        self.image, self.templates_z, self.scores_up = self._build_tracking_graph()
        self.final_score_sz = hp.response_up * (design.score_sz - 1) + 1

        scale_num = self._hyper_params.scale_num
        scale_step = self._hyper_params.scale_step
        self.scale_factors = scale_step**np.linspace(-np.ceil(scale_num/2), np.ceil(scale_num/2),
                                                     scale_num)

        # cosine window to penalize large displacements
        self.hann_1d = np.expand_dims(np.hanning(self.final_score_sz), axis=0)
        penalty = np.transpose(self.hann_1d) * self.hann_1d
        self.penalty = penalty / np.sum(penalty)

    def set_target(image, bbox):
        """Set target by providing a picture of the target and bounding box around target.

        Args:
            image (numpy.ndarray of dtype float): An image of the target. Presumed to be the first
                in a video sequence.
            bbox: (int, int, int, int): Ground truth bounding box around target in (x, y, w, h)
                format. x and y are image coordinates of the top left corner of the bounding box.
                w and h are the width and height of the bounding box respectively.

        Returns:
            None
        """

        pos_x, pos_y, target_w, target_h = bbox

        self.location = bbox
        self.context = self._design_params.context*(target_w+target_h)
        self.z_sz = np.sqrt(np.prod((target_w+self.context)*(target_h+self.context)))
        self.x_sz = float(self._design_params.search_sz) / self._design_params.exemplar_sz * z_sz

        # thresholds to saturate patches shrinking/growing
        self.min_z = self._hyper_params.scale_min * self.z_sz
        self.max_z = self._hyper_params.scale_max * self.z_sz
        self.min_x = self._hyper_params.scale_min * self.x_sz
        self.max_x = self._hyper_params.scale_max * self.x_sz

        # Extract target features and hold in memory.
        with tf.Session() as sess:
            tf.initialize_all_variables().run()
            self.template = np.squeeze(sess.run(templates_z, feed_dict={self.pos_x_ph: pos_x,
                                                                        self.pos_y_ph: pos_y,
                                                                        self.z_sz_ph: z_sz,
                                                                        image: image}))

    def detect(image):
        """Get bounding box for target in next image.

        Args:
            image (numpy.ndarray dtype=float32): The image in which to search for the target.

        Returns:
            Bounding box: A tuple of the form (int, int, int, int) with the following interpretation
                (x, y, width, height) where x and y are the top left corner of the bounding box.
        """

        # Variable set up.
        pos_x, pos_y, target_w, target_h = self.location
        scaled_exemplar = self.z_sz * self.scale_factors
        scaled_search_area = self.x_sz * self.scale_factors
        scaled_target_w = target_w * self.scale_factors
        scaled_target_h = target_h * self.scale_factors
        scale_lr = self._hyper_params.scale_lr
        scale_penalty = self._hyper_params.scale_penalty
        window_influence = self._hyper_params.window_influence

        # Run forward inference to get score map for search areas.
        scores_ = sess.run(
            [scores],
            feed_dict={
                self.pos_x_ph: pos_x,
                self.pos_y_ph: pos_y,
                self.x_sz0_ph: scaled_search_area[0],
                self.x_sz1_ph: scaled_search_area[1],
                self.x_sz2_ph: scaled_search_area[2],
                templates_z: self.template,
                image: image}, **run_opts)
        scores_ = np.squeeze(scores_)

        # penalize change of scale
        scores_[0,:,:] = scale_penalty*scores_[0,:,:]
        scores_[2,:,:] = scale_penalty*scores_[2,:,:]

        # find scale with highest peak (after penalty)
        new_scale_id = np.argmax(np.amax(scores_, axis=(1,2)))

        # update scaled sizes
        x_sz = (1-scale_lr)*x_sz + scale_lr*scaled_search_area[new_scale_id]
        target_w = (1-scale_lr)*target_w + scale_lr*scaled_target_w[new_scale_id]
        target_h = (1-scale_lr)*target_h + scale_lr*scaled_target_h[new_scale_id]

        # select response with new_scale_id
        score_ = scores_[new_scale_id,:,:]

        # normalize scores. TODO: Not exactly sure why we're doing this.
        score_ = score_ - np.min(score_)
        score_ = score_/np.sum(score_)

        # apply displacement penalty
        score_ = (1-window_influence) * score_ + window_influence * self.penalty
        pos_x, pos_y = _update_target_position(pos_x, pos_y, score_)

        # convert <cx,cy,w,h> to <x,y,w,h> and save output
        self.location = pos_x-target_w/2, pos_y-target_h/2, target_w, target_h

        # update template patch size
        z_sz = (1-scale_lr)*self.z_sz + scale_lr*scaled_exemplar[new_scale_id]

    def _update_target_position(self, pos_x, pos_y, score):
        # find location of score maximizer
        p = np.asarray(np.unravel_index(np.argmax(score), np.shape(score)))

        # displacement from the center in search area final representation ...
        center = float(self.final_score_sz - 1) / 2
        disp_in_area = p - center

        # displacement from the center in instance crop
        disp_in_xcrop = disp_in_area * float(self._design_params.tot_stride) / self._hyper_params.response_up

        # displacement from the center in instance crop (in frame coordinates)
        disp_in_frame = disp_in_xcrop *  self.x_sz / self._design_params.search_sz

        # *position* within frame in frame coordinates
        new_pos_y, new_pos_x = pos_y + disp_in_frame[0], pos_x + disp_in_frame[1]

        return new_pos_x, new_pos_y

    def _load_json_file(filename):
        with open(filename) as json_file:
            return json.load(json_file)

    def _build_tracking_graph():

        score_map_dim = self._hyper_params.response up * (self._design_params.score_sz - 1) + 1
        image = tf.placeholder(dtype=tf.float32, shape=(None,None,None), name='image')
        frame_sz = tf.shape(image)
        avg_chan = tf.reduce_mean(image, reduction_indices=(0,1), name='avg_chan')

        # pad with if necessary
        frame_padded_z, npad_z = _pad_frame(image, frame_sz, self.pos_x_ph, self.pos_y_ph,
                                            self.z_sz_ph, avg_chan)
        frame_padded_z = tf.cast(frame_padded_z, tf.float32)

        # extract tensor of z_crops
        z_crops = _extract_crops_z(frame_padded_z, npad_z, self.pos_x_ph, self.pos_y_ph,
                                   self.z_sz_ph,self._design_params.exemplar_sz)
        frame_padded_x, npad_x = _pad_frame(image, frame_sz, self.pos_x_ph, self.pos_y_ph,
                                            self.x_sz2_ph, avg_chan)
        frame_padded_x = tf.cast(frame_padded_x, tf.float32)

        # extract tensor of x_crops (3 scales)
        x_crops = _extract_crops_x(frame_padded_x, npad_x, self.pos_x_ph, self.pos_y_ph,
                                   self.x_sz0_ph, self.x_sz1_ph, self.x_sz2_ph,
                                   self._design_params.search_sz)

        # use crops as input of (MatConvnet imported) pre-trained fully-convolutional Siamese net
        weights_path = os.path.join(self._environment_params.root_pretrained,
                                    self._design_params.net)
        template_z, templates_x, p_names_list, p_val_list = _create_siamese(weights_path, x_crops,
                                                                            z_crops)
        template_z = tf.squeeze(template_z)
        templates_z = tf.pack([template_z, template_z, template_z])

        # compare templates via cross-correlation
        scores = _match_templates(templates_z, templates_x, p_names_list, p_val_list)
        # upsample the score maps
        scores_up = tf.image.resize_images(scores, [score_map_dim, score_map_dim],
            method=tf.image.ResizeMethod.BICUBIC, align_corners=True)
        return image, templates_z, scores_up

    def _pad_frame(im, frame_sz, pos_x, pos_y, patch_sz, avg_chan):
        c = patch_sz / 2
        xleft_pad = tf.maximum(0, -tf.cast(tf.round(pos_x - c), tf.int32))
        ytop_pad = tf.maximum(0, -tf.cast(tf.round(pos_y - c), tf.int32))
        xright_pad = tf.maximum(0, tf.cast(tf.round(pos_x + c), tf.int32) - frame_sz[1])
        ybottom_pad = tf.maximum(0, tf.cast(tf.round(pos_y + c), tf.int32) - frame_sz[0])
        npad = tf.reduce_max([xleft_pad, ytop_pad, xright_pad, ybottom_pad])
        paddings = [[npad, npad], [npad, npad], [0, 0]]
        im_padded = im
        if avg_chan is not None:
            im_padded = im_padded - avg_chan
        im_padded = tf.pad(im_padded, paddings, mode='CONSTANT')
        if avg_chan is not None:
            im_padded = im_padded + avg_chan
        return im_padded, npad

    def _extract_crops_z(im, npad, pos_x, pos_y, sz_src, sz_dst):
        c = sz_src / 2
        # get top-right corner of bbox and consider padding
        tr_x = npad + tf.cast(tf.round(pos_x - c), tf.int32)
        # Compute size from rounded co-ords to ensure rectangle lies inside padding.
        tr_y = npad + tf.cast(tf.round(pos_y - c), tf.int32)
        width = tf.round(pos_x + c) - tf.round(pos_x - c)
        height = tf.round(pos_y + c) - tf.round(pos_y - c)
        crop = tf.image.crop_to_bounding_box(im,
                                             tf.cast(tr_y, tf.int32),
                                             tf.cast(tr_x, tf.int32),
                                             tf.cast(height, tf.int32),
                                             tf.cast(width, tf.int32))
        crop = tf.image.resize_images(crop, [sz_dst, sz_dst], method=tf.image.ResizeMethod.BILINEAR)
        # crops = tf.stack([crop, crop, crop])
        crops = tf.expand_dims(crop, dim=0)
        return crops

    def _extract_crops_x(im, npad, pos_x, pos_y, sz_src0, sz_src1, sz_src2, sz_dst):
        # take center of the biggest scaled source patch
        c = sz_src2 / 2
        # get top-right corner of bbox and consider padding
        tr_x = npad + tf.cast(tf.round(pos_x - c), tf.int32)
        tr_y = npad + tf.cast(tf.round(pos_y - c), tf.int32)
        # Compute size from rounded co-ords to ensure rectangle lies inside padding.
        width = tf.round(pos_x + c) - tf.round(pos_x - c)
        height = tf.round(pos_y + c) - tf.round(pos_y - c)
        search_area = tf.image.crop_to_bounding_box(im,
                                                    tf.cast(tr_y, tf.int32),
                                                    tf.cast(tr_x, tf.int32),
                                                    tf.cast(height, tf.int32),
                                                    tf.cast(width, tf.int32))
        # TODO: Use computed width and height here?
        offset_s0 = (sz_src2 - sz_src0) / 2
        offset_s1 = (sz_src2 - sz_src1) / 2

        crop_s0 = tf.image.crop_to_bounding_box(search_area,
                                                tf.cast(offset_s0, tf.int32),
                                                tf.cast(offset_s0, tf.int32),
                                                tf.cast(tf.round(sz_src0), tf.int32),
                                                tf.cast(tf.round(sz_src0), tf.int32))
        crop_s0 = tf.image.resize_images(crop_s0, [sz_dst, sz_dst], method=tf.image.ResizeMethod.
                                         BILINEAR)
        crop_s1 = tf.image.crop_to_bounding_box(search_area,
                                                tf.cast(offset_s1, tf.int32),
                                                tf.cast(offset_s1, tf.int32),
                                                tf.cast(tf.round(sz_src1), tf.int32),
                                                tf.cast(tf.round(sz_src1), tf.int32))
        crop_s1 = tf.image.resize_images(crop_s1, [sz_dst, sz_dst], method=tf.image.ResizeMethod.
                                         BILINEAR)
        crop_s2 = tf.image.resize_images(search_area, [sz_dst, sz_dst], method=tf.image.
                                         ResizeMethod.BILINEAR)
        crops = tf.pack([crop_s0, crop_s1, crop_s2])
        return crops

    # import pretrained Siamese network from matconvnet
    def _create_siamese(net_path, net_x, net_z):
        # read mat file from net_path and start TF Siamese graph from placeholders X and Z
        params_names_list, params_values_list = _import_from_matconvnet(net_path)

        # loop through the flag arrays and re-construct network, reading parameters of conv and
        # bnorm layers
        for i in xrange(_num_layers):
            print '> Layer '+str(i+1)
            # conv
            conv_W_name = _find_params('conv'+str(i+1)+'f', params_names_list)[0]
            conv_b_name = _find_params('conv'+str(i+1)+'b', params_names_list)[0]
            print '\t\tCONV: setting '+conv_W_name+' '+conv_b_name
            print '\t\tCONV: stride '+str(_conv_stride[i])+', filter-group '+str(_filtergroup_yn[i])
            conv_W = params_values_list[params_names_list.index(conv_W_name)]
            conv_b = params_values_list[params_names_list.index(conv_b_name)]
            # batchnorm
            if _bnorm_yn[i]:
                bn_beta_name = _find_params('bn'+str(i+1)+'b', params_names_list)[0]
                bn_gamma_name = _find_params('bn'+str(i+1)+'m', params_names_list)[0]
                bn_moments_name = _find_params('bn'+str(i+1)+'x', params_names_list)[0]
                print '\t\tBNORM: setting '+bn_beta_name+' '+bn_gamma_name+' '+bn_moments_name
                bn_beta = params_values_list[params_names_list.index(bn_beta_name)]
                bn_gamma = params_values_list[params_names_list.index(bn_gamma_name)]
                bn_moments = params_values_list[params_names_list.index(bn_moments_name)]
                bn_moving_mean = bn_moments[:,0]
                bn_moving_variance = bn_moments[:,1]**2 # saved as std in matconvnet
            else:
                bn_beta = bn_gamma = bn_moving_mean = bn_moving_variance = []

            # set up conv "block" with bnorm and activation
            net_x = _set_convolutional(net_x, conv_W, np.swapaxes(conv_b,0,1), _conv_stride[i], \
                                bn_beta, bn_gamma, bn_moving_mean, bn_moving_variance, \
                                filtergroup=_filtergroup_yn[i], batchnorm=_bnorm_yn[i], \
                                activation=_relu_yn[i], scope='conv'+str(i+1), reuse=False)

            # notice reuse=True for Siamese parameters sharing
            net_z = _set_convolutional(net_z, conv_W, np.swapaxes(conv_b,0,1), _conv_stride[i], \
                                bn_beta, bn_gamma, bn_moving_mean, bn_moving_variance, \
                                filtergroup=_filtergroup_yn[i], batchnorm=_bnorm_yn[i], \
                                activation=_relu_yn[i], scope='conv'+str(i+1), reuse=True)

            # add max pool if required
            if _pool_stride[i]>0:
                print '\t\tMAX-POOL: size '+str(_pool_sz)+ ' and stride '+str(_pool_stride[i])
                net_x = tf.nn.max_pool(net_x, [1,_pool_sz,_pool_sz,1], strides=[1,_pool_stride[i],
                                       _pool_stride[i],1], padding='VALID', name='pool'+str(i+1))
                net_z = tf.nn.max_pool(net_z, [1,_pool_sz,_pool_sz,1], strides=[1,_pool_stride[i],
                                       _pool_stride[i],1], padding='VALID', name='pool'+str(i+1))

        print

        return net_z, net_x, params_names_list, params_values_list

    def _set_convolutional(X, W, b, stride, bn_beta, bn_gamma, bn_mm, bn_mv, filtergroup=False,
                          batchnorm=True, activation=True, scope=None, reuse=False):

        # use the input scope or default to "conv"
        with tf.variable_scope(scope or 'conv', reuse=reuse):
            # sanity check
            print "W SHAPE: {0}".format(W.shape)
            print "X SHAPE: {0}".format(X.get_shape())
            W = tf.get_variable("W", W.shape, trainable=False,
                                initializer=tf.constant_initializer(W))
            b = tf.get_variable("b", b.shape, trainable=False,
                                initializer=tf.constant_initializer(b))

            # "filtergroup" Is a bizarre name for this variable. This is a holdover from the
            # original alexnet, and is used to split the network across two separate GPUs. The
            # input tensor is split along the in_channel axis, and the filter is
            # split along out_channel axis (in both cases this ends up being dimension 4, which is
            # why the leading '3' argument to tf.split. The '2' arg splits the tensor into 2 pieces
            # (surprise).
            # What remains unclear is how and why the channels argument ends up coming first on the
            # original input tensor (the image). While the documentation for tf r0.11 and r1.3 say
            # that the input tensor format must be (batch_size, width, height, channels), printing
            # the input tensor gives (3, 255, 255, ?) suggesting that the batch_size is at the end.
            # It is also unclear how tensorflow resolves the issue since during inference
            # batch_size = 1, which seems like it would break the convolution.
            if filtergroup:
                X0, X1 = tf.split(3, 2, X)
                W0, W1 = tf.split(3, 2, W)
                h0 = tf.nn.conv2d(X0, W0, strides=[1, stride, stride, 1], padding='VALID')
                h1 = tf.nn.conv2d(X1, W1, strides=[1, stride, stride, 1], padding='VALID')
                h = tf.concat(3, [h0, h1]) + b
            else:
                h = tf.nn.conv2d(X, W, strides=[1, stride, stride, 1], padding='VALID') + b

            if batchnorm:
                param_initializer = {
                    'beta': tf.constant_initializer(bn_beta),
                    'gamma':  tf.constant_initializer(bn_gamma),
                    'moving_mean': tf.constant_initializer(bn_mm),
                    'moving_variance': tf.constant_initializer(bn_mv)
                }
                h = tf.contrib.layers.batch_norm(h, initializers=param_initializer,
                                                  is_training=False, trainable=False)

            if activation:
                h = tf.nn.relu(h)

            return h

    def _import_from_matconvnet(net_path):
        mat = scipy.io.loadmat(net_path)
        net_dot_mat = mat.get('net')
        # organize parameters to import
        params = net_dot_mat['params']
        params = params[0][0]
        params_names = params['name'][0]
        params_names_list = [params_names[p][0] for p in xrange(params_names.size)]
        params_values = params['value'][0]
        params_values_list = [params_values[p] for p in xrange(params_values.size)]
        return params_names_list, params_values_list


    # find all parameters matching the codename (there should be only one)
    def _find_params(x, params):
        matching = [s for s in params if x in s]
        assert len(matching)==1, ('Ambiguous param name found')
        return matching


    def _match_templates(net_z, net_x, params_names_list, params_values_list):
        # finalize network
        # z, x are [B, H, W, C]
        net_z = tf.transpose(net_z, perm=[1,2,0,3])
        net_x = tf.transpose(net_x, perm=[1,2,0,3])
        # z, x are [H, W, B, C]
        Hz, Wz, B, C = tf.unpack(tf.shape(net_z))
        Hx, Wx, Bx, Cx = tf.unpack(tf.shape(net_x))
        # assert B==Bx, ('Z and X should have same Batch size')
        # assert C==Cx, ('Z and X should have same Channels number')
        net_z = tf.reshape(net_z, (Hz, Wz, B*C, 1))
        net_x = tf.reshape(net_x, (1, Hx, Wx, B*C))
        net_final = tf.nn.depthwise_conv2d(net_x, net_z, strides=[1,1,1,1], padding='VALID')
        # final is [1, Hf, Wf, BC]
        net_final = tf.concat(0, tf.split(3, 3, net_final))
        # final is [B, Hf, Wf, C]
        net_final = tf.expand_dims(tf.reduce_sum(net_final, reduction_indices=3), dim=3)
        # final is [B, Hf, Wf, 1]
        if _bnorm_adjust:
            bn_beta = params_values_list[params_names_list.index('fin_adjust_bnb')]
            bn_gamma = params_values_list[params_names_list.index('fin_adjust_bnm')]
            bn_moments = params_values_list[params_names_list.index('fin_adjust_bnx')]
            bn_moving_mean = bn_moments[:,0]
            bn_moving_variance = bn_moments[:,1]**2
            param_initializer = {
                'beta': tf.constant_initializer(bn_beta),
                'gamma':  tf.constant_initializer(bn_gamma),
                'moving_mean': tf.constant_initializer(bn_moving_mean),
                'moving_variance': tf.constant_initializer(bn_moving_variance)
            }
            net_final = tf.contrib.layers.batch_norm(net_final, initializers=param_initializer,
                                                     is_training=False, trainable=False)


        return net_final
