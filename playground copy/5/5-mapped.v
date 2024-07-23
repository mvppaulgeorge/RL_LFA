// Benchmark "adder" written by ABC on Thu Jul 11 11:12:41 2024

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
    new_n125, new_n126, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n312, new_n313,
    new_n315;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  aoi112aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n106));
  160nm_fiao0012aa1n02p5x5     g011(.a(new_n100), .b(new_n102), .c(new_n101), .o(new_n107));
  oaoi13aa1n02x5               g012(.a(new_n107), .b(new_n104), .c(new_n105), .d(new_n106), .o1(new_n108));
  xorc02aa1n02x5               g013(.a(\a[6] ), .b(\b[5] ), .out0(new_n109));
  xorc02aa1n02x5               g014(.a(\a[5] ), .b(\b[4] ), .out0(new_n110));
  nanp03aa1n02x5               g015(.a(new_n104), .b(new_n109), .c(new_n110), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(new_n116), .clkout(new_n117));
  nanp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n117), .o1(new_n121));
  oab012aa1n02x4               g026(.a(new_n113), .b(\a[4] ), .c(\b[3] ), .out0(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(new_n122), .clkout(new_n123));
  aoai13aa1n02x5               g028(.a(new_n112), .b(new_n123), .c(new_n121), .d(new_n115), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n108), .b(new_n124), .c(new_n111), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n98), .b(new_n99), .c(new_n125), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g032(.a(\a[10] ), .b(\b[9] ), .c(new_n126), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norp02aa1n02x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n130), .c(new_n128), .d(new_n132), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n128), .b(new_n132), .o1(new_n137));
  nona22aa1n02x4               g042(.a(new_n137), .b(new_n135), .c(new_n130), .out0(new_n138));
  nanp02aa1n02x5               g043(.a(new_n138), .b(new_n136), .o1(\s[12] ));
  nano23aa1n02x4               g044(.a(new_n130), .b(new_n133), .c(new_n134), .d(new_n131), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n99), .b(new_n98), .o1(new_n141));
  oaoi03aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(new_n141), .o1(new_n142));
  160nm_fiao0012aa1n02p5x5     g047(.a(new_n133), .b(new_n130), .c(new_n134), .o(new_n143));
  aoi012aa1n02x5               g048(.a(new_n143), .b(new_n140), .c(new_n142), .o1(new_n144));
  nona23aa1n02x4               g049(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n145));
  norp02aa1n02x5               g050(.a(new_n106), .b(new_n105), .o1(new_n146));
  oabi12aa1n02x5               g051(.a(new_n107), .b(new_n145), .c(new_n146), .out0(new_n147));
  nano22aa1n02x4               g052(.a(new_n145), .b(new_n109), .c(new_n110), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n115), .b(new_n116), .c(new_n119), .d(new_n118), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n122), .c(\b[3] ), .d(\a[4] ), .o1(new_n150));
  xorc02aa1n02x5               g055(.a(\a[9] ), .b(\b[8] ), .out0(new_n151));
  nanp03aa1n02x5               g056(.a(new_n140), .b(new_n97), .c(new_n151), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n147), .c(new_n150), .d(new_n148), .o1(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n154), .c(new_n144), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g062(.clk(\a[14] ), .clkout(new_n158));
  nanp02aa1n02x5               g063(.a(new_n154), .b(new_n144), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n156), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  xnrc02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  norp02aa1n02x5               g068(.a(new_n163), .b(new_n155), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(\b[13] ), .clkout(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n158), .b(new_n166), .c(new_n160), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n154), .d(new_n144), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n171), .clkout(new_n172));
  xnrc02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n170), .c(new_n168), .d(new_n172), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n168), .b(new_n172), .o1(new_n175));
  nona22aa1n02x4               g080(.a(new_n175), .b(new_n173), .c(new_n170), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n174), .o1(\s[16] ));
  orn002aa1n02x5               g082(.a(\a[15] ), .b(\b[14] ), .o(new_n178));
  oao003aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n179));
  oai013aa1n02x4               g084(.a(new_n179), .b(new_n167), .c(new_n171), .d(new_n173), .o1(new_n180));
  norp02aa1n02x5               g085(.a(new_n173), .b(new_n171), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n164), .o1(new_n182));
  oab012aa1n02x4               g087(.a(new_n180), .b(new_n182), .c(new_n144), .out0(new_n183));
  nano22aa1n02x4               g088(.a(new_n152), .b(new_n164), .c(new_n181), .out0(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n147), .c(new_n150), .d(new_n148), .o1(new_n185));
  xorc02aa1n02x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  xnbna2aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n183), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[17] ), .clkout(new_n189));
  aobi12aa1n02x5               g094(.a(new_n186), .b(new_n185), .c(new_n183), .out0(new_n190));
  aoib12aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(\b[16] ), .out0(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d04x5               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  norp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n196));
  norp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n183), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  norp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n206), .clkout(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n201), .c(new_n198), .d(new_n203), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n198), .b(new_n203), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n207), .c(new_n201), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n208), .o1(\s[20] ));
  nona23aa1n02x4               g116(.a(new_n205), .b(new_n202), .c(new_n201), .d(new_n204), .out0(new_n212));
  aoi012aa1n02x5               g117(.a(new_n204), .b(new_n201), .c(new_n205), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n212), .c(new_n197), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  nanb02aa1n02x5               g120(.a(new_n212), .b(new_n193), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n216), .c(new_n185), .d(new_n183), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(new_n217), .b(new_n220), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n223), .b(new_n221), .c(new_n219), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  oai112aa1n02x5               g130(.a(new_n203), .b(new_n206), .c(new_n196), .d(new_n195), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n221), .b(new_n220), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\a[22] ), .clkout(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\b[21] ), .clkout(new_n229));
  oaoi03aa1n02x5               g134(.a(new_n228), .b(new_n229), .c(new_n219), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n226), .d(new_n213), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  nona23aa1n02x4               g137(.a(new_n193), .b(new_n220), .c(new_n221), .d(new_n212), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n232), .b(new_n233), .c(new_n185), .d(new_n183), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n234), .b(new_n237), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n240), .b(new_n238), .c(new_n236), .out0(new_n241));
  nanp02aa1n02x5               g146(.a(new_n241), .b(new_n239), .o1(\s[24] ));
  norb02aa1n02x5               g147(.a(new_n237), .b(new_n238), .out0(new_n243));
  nona23aa1n02x4               g148(.a(new_n193), .b(new_n243), .c(new_n227), .d(new_n212), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(\a[24] ), .clkout(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(\b[23] ), .clkout(new_n246));
  oaoi03aa1n02x5               g151(.a(new_n245), .b(new_n246), .c(new_n236), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n231), .c(new_n243), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n244), .c(new_n185), .d(new_n183), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n254), .clkout(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n250), .b(new_n253), .o1(new_n257));
  nona22aa1n02x4               g162(.a(new_n257), .b(new_n255), .c(new_n252), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(new_n258), .b(new_n256), .o1(\s[26] ));
  and002aa1n02x5               g164(.a(new_n254), .b(new_n253), .o(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n248), .c(new_n231), .d(new_n243), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n262));
  oab012aa1n02x4               g167(.a(new_n262), .b(\a[26] ), .c(\b[25] ), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n167), .clkout(new_n264));
  aobi12aa1n02x5               g169(.a(new_n179), .b(new_n181), .c(new_n264), .out0(new_n265));
  oai012aa1n02x5               g170(.a(new_n265), .b(new_n182), .c(new_n144), .o1(new_n266));
  nano22aa1n02x4               g171(.a(new_n233), .b(new_n243), .c(new_n260), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n266), .c(new_n125), .d(new_n184), .o1(new_n268));
  nanp03aa1n02x5               g173(.a(new_n268), .b(new_n261), .c(new_n263), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n261), .b(new_n263), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n267), .clkout(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n185), .c(new_n183), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n272), .b(new_n275), .c(new_n277), .o1(new_n278));
  nona22aa1n02x4               g183(.a(new_n278), .b(new_n273), .c(new_n271), .out0(new_n279));
  nanp02aa1n02x5               g184(.a(new_n274), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n272), .b(new_n273), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n275), .c(new_n277), .o1(new_n282));
  aob012aa1n02x5               g187(.a(new_n271), .b(\b[27] ), .c(\a[28] ), .out0(new_n283));
  oa0012aa1n02x5               g188(.a(new_n283), .b(\b[27] ), .c(\a[28] ), .o(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n284), .clkout(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n02x4               g191(.a(new_n282), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n286), .b(new_n285), .c(new_n269), .d(new_n281), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n272), .b(new_n286), .c(new_n273), .out0(new_n291));
  oaoi03aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .o1(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n292), .c(new_n269), .d(new_n291), .o1(new_n294));
  oai012aa1n02x5               g199(.a(new_n291), .b(new_n275), .c(new_n277), .o1(new_n295));
  nona22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  nanp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  nanb02aa1n02x5               g202(.a(new_n293), .b(new_n292), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n298), .b(\b[29] ), .c(\a[30] ), .o1(new_n299));
  norb02aa1n02x5               g204(.a(new_n291), .b(new_n293), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(new_n275), .c(new_n277), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nona22aa1n02x4               g207(.a(new_n301), .b(new_n302), .c(new_n299), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n302), .b(new_n299), .c(new_n269), .d(new_n300), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n304), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g210(.a(new_n115), .b(new_n120), .c(new_n117), .out0(\s[3] ));
  aoi012aa1n02x5               g211(.a(new_n113), .b(new_n121), .c(new_n115), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g213(.a(new_n124), .b(new_n110), .out0(\s[5] ));
  oaoi03aa1n02x5               g214(.a(\a[5] ), .b(\b[4] ), .c(new_n124), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanp02aa1n02x5               g216(.a(new_n110), .b(new_n109), .o1(new_n312));
  oai012aa1n02x5               g217(.a(new_n146), .b(new_n124), .c(new_n312), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n102), .b(new_n313), .c(new_n103), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g221(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


