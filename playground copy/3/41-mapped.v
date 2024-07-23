// Benchmark "adder" written by ABC on Wed Jul 10 16:56:37 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .out0(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oai012aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  oa0022aa1n02x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  oaoi13aa1n02x5               g012(.a(new_n101), .b(new_n107), .c(new_n102), .d(new_n106), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xorc02aa1n02x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  norb03aa1n02x5               g020(.a(new_n115), .b(new_n113), .c(new_n114), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(new_n108), .b(new_n116), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n119));
  and002aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .o(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\a[5] ), .clkout(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[6] ), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[4] ), .clkout(new_n123));
  aboi22aa1n03x5               g028(.a(\b[5] ), .b(new_n122), .c(new_n121), .d(new_n123), .out0(new_n124));
  nona22aa1n02x4               g029(.a(new_n119), .b(new_n120), .c(new_n124), .out0(new_n125));
  nona22aa1n02x4               g030(.a(new_n125), .b(new_n118), .c(new_n109), .out0(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  nona22aa1n02x4               g032(.a(new_n117), .b(new_n126), .c(new_n127), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n98), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aob012aa1n02x5               g035(.a(new_n97), .b(new_n128), .c(new_n100), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xobna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n130), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n132), .clkout(new_n136));
  nano22aa1n02x4               g041(.a(new_n132), .b(new_n130), .c(new_n133), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n98), .c(new_n128), .d(new_n100), .o1(new_n138));
  xorc02aa1n02x5               g043(.a(\a[12] ), .b(\b[11] ), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n138), .c(new_n136), .out0(\s[12] ));
  xorc02aa1n02x5               g045(.a(\a[13] ), .b(\b[12] ), .out0(new_n141));
  oai022aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n142));
  nona23aa1n02x4               g047(.a(new_n137), .b(new_n139), .c(new_n142), .d(new_n99), .out0(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n143), .clkout(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n126), .c(new_n108), .d(new_n116), .o1(new_n145));
  norp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n147));
  nanp03aa1n02x5               g052(.a(new_n137), .b(new_n139), .c(new_n142), .o1(new_n148));
  nona22aa1n02x4               g053(.a(new_n148), .b(new_n147), .c(new_n146), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n141), .b(new_n145), .c(new_n150), .out0(\s[13] ));
  orn002aa1n02x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  oai012aa1n02x5               g057(.a(new_n107), .b(new_n102), .c(new_n106), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n101), .b(new_n153), .out0(new_n154));
  nanb03aa1n02x5               g059(.a(new_n114), .b(new_n119), .c(new_n115), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n120), .clkout(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n124), .clkout(new_n157));
  aoi113aa1n02x5               g062(.a(new_n109), .b(new_n118), .c(new_n119), .d(new_n156), .e(new_n157), .o1(new_n158));
  oai012aa1n02x5               g063(.a(new_n158), .b(new_n154), .c(new_n155), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n141), .b(new_n149), .c(new_n159), .d(new_n144), .o1(new_n160));
  xorc02aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n152), .out0(\s[14] ));
  nanp02aa1n02x5               g067(.a(new_n161), .b(new_n141), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n145), .d(new_n150), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nanp02aa1n02x5               g078(.a(new_n170), .b(new_n169), .o1(new_n174));
  norp03aa1n02x5               g079(.a(new_n143), .b(new_n163), .c(new_n174), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n126), .c(new_n108), .d(new_n116), .o1(new_n176));
  nano22aa1n02x4               g081(.a(new_n163), .b(new_n169), .c(new_n170), .out0(new_n177));
  norp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  nanp03aa1n02x5               g084(.a(new_n164), .b(new_n169), .c(new_n170), .o1(new_n180));
  nona22aa1n02x4               g085(.a(new_n180), .b(new_n179), .c(new_n178), .out0(new_n181));
  aoi012aa1n02x5               g086(.a(new_n181), .b(new_n149), .c(new_n177), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n176), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  oai022aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n02x5               g096(.a(new_n191), .b(new_n185), .c(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x4               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  orn002aa1n02x5               g109(.a(\a[19] ), .b(\b[18] ), .o(new_n205));
  aobi12aa1n02x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  nona23aa1n02x4               g114(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n210), .c(new_n192), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n209), .c(new_n176), .d(new_n182), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n190), .c(new_n208), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oao003aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n213), .c(new_n225), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n176), .d(new_n182), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n225), .c(new_n190), .d(new_n208), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n225), .b(new_n211), .c(new_n208), .d(new_n193), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n228), .clkout(new_n242));
  oai022aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n243));
  aob012aa1n02x5               g148(.a(new_n243), .b(\b[23] ), .c(\a[24] ), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n239), .c(new_n241), .d(new_n242), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n240), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n240), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x4               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n250), .clkout(new_n253));
  aobi12aa1n02x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  nanp02aa1n02x5               g160(.a(new_n149), .b(new_n177), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n181), .clkout(new_n257));
  nanp02aa1n02x5               g162(.a(new_n256), .b(new_n257), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(\a[25] ), .clkout(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(\a[26] ), .clkout(new_n260));
  xroi22aa1d04x5               g165(.a(new_n259), .b(\b[24] ), .c(new_n260), .d(\b[25] ), .out0(new_n261));
  nano22aa1n02x4               g166(.a(new_n226), .b(new_n238), .c(new_n261), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n258), .c(new_n159), .d(new_n175), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n245), .c(new_n261), .out0(new_n265));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  and002aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o(new_n267));
  norp02aa1n02x5               g172(.a(new_n267), .b(new_n266), .o1(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n265), .c(new_n263), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n267), .clkout(new_n270));
  xorc02aa1n02x5               g175(.a(\a[28] ), .b(\b[27] ), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n238), .b(new_n228), .c(new_n213), .d(new_n225), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n261), .clkout(new_n273));
  aoai13aa1n02x5               g178(.a(new_n264), .b(new_n273), .c(new_n272), .d(new_n244), .o1(new_n274));
  aoi112aa1n02x5               g179(.a(new_n274), .b(new_n266), .c(new_n183), .d(new_n262), .o1(new_n275));
  nano22aa1n02x4               g180(.a(new_n275), .b(new_n270), .c(new_n271), .out0(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n266), .clkout(new_n277));
  nanp03aa1n02x5               g182(.a(new_n265), .b(new_n263), .c(new_n277), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n271), .b(new_n278), .c(new_n270), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n276), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n262), .clkout(new_n281));
  aoi012aa1n02x5               g186(.a(new_n281), .b(new_n176), .c(new_n182), .o1(new_n282));
  and002aa1n02x5               g187(.a(new_n271), .b(new_n268), .o(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n274), .c(new_n282), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n283), .b(new_n265), .c(new_n263), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g196(.a(new_n286), .b(new_n271), .c(new_n268), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n274), .c(new_n282), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x5               g201(.a(new_n292), .b(new_n265), .c(new_n263), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nano23aa1n02x4               g204(.a(new_n295), .b(new_n286), .c(new_n271), .d(new_n268), .out0(new_n300));
  aobi12aa1n02x5               g205(.a(new_n300), .b(new_n265), .c(new_n263), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n300), .b(new_n274), .c(new_n282), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g216(.a(new_n121), .b(new_n123), .c(new_n108), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(new_n122), .out0(\s[6] ));
  oaoi03aa1n02x5               g218(.a(\a[6] ), .b(\b[5] ), .c(new_n312), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n111), .b(new_n314), .c(new_n112), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g222(.a(new_n127), .b(new_n117), .c(new_n158), .out0(\s[9] ));
endmodule


