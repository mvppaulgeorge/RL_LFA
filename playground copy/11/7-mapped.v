// Benchmark "adder" written by ABC on Thu Jul 11 11:40:26 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n343, new_n344, new_n346, new_n348;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanb03aa1n02x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .out0(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\a[3] ), .clkout(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\b[2] ), .clkout(new_n108));
  nanp02aa1n02x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(new_n110), .clkout(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nona22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[6] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[5] ), .clkout(new_n121));
  norp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oao003aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n123));
  160nm_fiao0012aa1n02p5x5     g028(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n119), .c(new_n106), .d(new_n111), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g034(.clk(\a[4] ), .clkout(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(\b[3] ), .clkout(new_n131));
  nanp02aa1n02x5               g036(.a(new_n131), .b(new_n130), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[3] ), .b(\a[4] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n132), .b(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n109), .b(new_n104), .o1(new_n135));
  norp03aa1n02x5               g040(.a(new_n101), .b(new_n134), .c(new_n135), .o1(new_n136));
  nona23aa1n02x4               g041(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n137));
  norp03aa1n02x5               g042(.a(new_n137), .b(new_n117), .c(new_n118), .o1(new_n138));
  oai012aa1n02x5               g043(.a(new_n138), .b(new_n136), .c(new_n110), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nano23aa1n02x4               g046(.a(new_n140), .b(new_n97), .c(new_n127), .d(new_n141), .out0(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n142), .clkout(new_n143));
  aoi012aa1n02x5               g048(.a(new_n140), .b(new_n97), .c(new_n141), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n139), .d(new_n125), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nano23aa1n02x4               g057(.a(new_n147), .b(new_n151), .c(new_n152), .d(new_n148), .out0(new_n153));
  nona23aa1n02x4               g058(.a(new_n152), .b(new_n148), .c(new_n147), .d(new_n151), .out0(new_n154));
  aoi012aa1n02x5               g059(.a(new_n151), .b(new_n147), .c(new_n152), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n144), .o1(new_n156));
  aoi013aa1n02x4               g061(.a(new_n156), .b(new_n126), .c(new_n142), .d(new_n153), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n157), .b(new_n160), .c(new_n159), .out0(\s[13] ));
  nanb02aa1n02x5               g066(.a(new_n158), .b(new_n160), .out0(new_n162));
  norp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  oai112aa1n02x5               g071(.a(new_n166), .b(new_n159), .c(new_n157), .d(new_n162), .o1(new_n167));
  oaoi13aa1n02x5               g072(.a(new_n166), .b(new_n159), .c(new_n157), .d(new_n162), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[14] ));
  nano23aa1n02x4               g074(.a(new_n158), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n170));
  aoi012aa1n02x5               g075(.a(new_n163), .b(new_n158), .c(new_n164), .o1(new_n171));
  aobi12aa1n02x5               g076(.a(new_n171), .b(new_n156), .c(new_n170), .out0(new_n172));
  nona23aa1n02x4               g077(.a(new_n164), .b(new_n160), .c(new_n158), .d(new_n163), .out0(new_n173));
  nona32aa1n02x4               g078(.a(new_n126), .b(new_n173), .c(new_n154), .d(new_n143), .out0(new_n174));
  norp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  xobna2aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n172), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n175), .clkout(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n174), .d(new_n172), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  norp02aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n183), .b(new_n176), .c(new_n175), .d(new_n182), .out0(new_n184));
  norp02aa1n02x5               g089(.a(new_n184), .b(new_n173), .o1(new_n185));
  nanp03aa1n02x5               g090(.a(new_n185), .b(new_n142), .c(new_n153), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n182), .b(new_n175), .c(new_n183), .o1(new_n187));
  oai012aa1n02x5               g092(.a(new_n187), .b(new_n184), .c(new_n171), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n156), .c(new_n185), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n186), .c(new_n139), .d(new_n125), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[18] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[16] ), .clkout(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(new_n192), .out0(\s[18] ));
  oai013aa1n02x4               g101(.a(new_n111), .b(new_n134), .c(new_n101), .d(new_n135), .o1(new_n197));
  oaoi03aa1n02x5               g102(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n198));
  oabi12aa1n02x5               g103(.a(new_n124), .b(new_n137), .c(new_n198), .out0(new_n199));
  nano23aa1n02x4               g104(.a(new_n175), .b(new_n182), .c(new_n183), .d(new_n176), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n170), .o1(new_n201));
  nano22aa1n02x4               g106(.a(new_n201), .b(new_n142), .c(new_n153), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n197), .d(new_n138), .o1(new_n203));
  xroi22aa1d04x5               g108(.a(new_n193), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  norp02aa1n02x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  norp02aa1n02x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n207), .b(new_n206), .c(new_n208), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n205), .c(new_n203), .d(new_n189), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[19] ), .b(\b[18] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[20] ), .b(\b[19] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n210), .d(new_n214), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n213), .clkout(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n209), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n190), .d(new_n204), .o1(new_n219));
  xnrc02aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .out0(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n219), .c(new_n217), .o1(new_n221));
  norp02aa1n02x5               g126(.a(new_n221), .b(new_n216), .o1(\s[20] ));
  xnrc02aa1n02x5               g127(.a(\b[18] ), .b(\a[19] ), .out0(new_n223));
  norp02aa1n02x5               g128(.a(new_n220), .b(new_n223), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n204), .b(new_n224), .o1(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[20] ), .b(\b[19] ), .c(new_n217), .o1(new_n226));
  aoi013aa1n02x4               g131(.a(new_n226), .b(new_n218), .c(new_n214), .d(new_n215), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n227), .b(new_n225), .c(new_n203), .d(new_n189), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nanb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  norp02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoi112aa1n02x5               g142(.a(new_n230), .b(new_n237), .c(new_n228), .d(new_n233), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n230), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n225), .clkout(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n227), .clkout(new_n241));
  aoai13aa1n02x5               g146(.a(new_n233), .b(new_n241), .c(new_n190), .d(new_n240), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n236), .b(new_n242), .c(new_n239), .o1(new_n243));
  norp02aa1n02x5               g148(.a(new_n243), .b(new_n238), .o1(\s[22] ));
  nanb03aa1n02x5               g149(.a(new_n209), .b(new_n215), .c(new_n214), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n226), .clkout(new_n246));
  nona23aa1n02x4               g151(.a(new_n235), .b(new_n231), .c(new_n230), .d(new_n234), .out0(new_n247));
  aoi012aa1n02x5               g152(.a(new_n234), .b(new_n230), .c(new_n235), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n245), .d(new_n246), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n247), .clkout(new_n251));
  nanp03aa1n02x5               g156(.a(new_n204), .b(new_n251), .c(new_n224), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n250), .b(new_n252), .c(new_n203), .d(new_n189), .o1(new_n253));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n252), .clkout(new_n257));
  aoi112aa1n02x5               g162(.a(new_n249), .b(new_n256), .c(new_n190), .d(new_n257), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n256), .o1(\s[23] ));
  norp02aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n254), .b(new_n262), .c(new_n253), .d(new_n256), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n254), .clkout(new_n264));
  aoai13aa1n02x5               g169(.a(new_n256), .b(new_n249), .c(new_n190), .d(new_n257), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n262), .clkout(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .o1(new_n267));
  norp02aa1n02x5               g172(.a(new_n267), .b(new_n263), .o1(\s[24] ));
  nano23aa1n02x4               g173(.a(new_n254), .b(new_n260), .c(new_n261), .d(new_n255), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n225), .b(new_n251), .c(new_n269), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  nona22aa1n02x4               g176(.a(new_n269), .b(new_n236), .c(new_n232), .out0(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .o1(new_n273));
  aoib12aa1n02x5               g178(.a(new_n273), .b(new_n269), .c(new_n248), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n272), .c(new_n245), .d(new_n246), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n203), .d(new_n189), .o1(new_n277));
  xorb03aa1n02x5               g182(.a(new_n277), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  aoi112aa1n02x5               g186(.a(new_n279), .b(new_n281), .c(new_n277), .d(new_n280), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n279), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n280), .b(new_n275), .c(new_n190), .d(new_n270), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n281), .clkout(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n144), .clkout(new_n288));
  aobi12aa1n02x5               g193(.a(new_n155), .b(new_n153), .c(new_n288), .out0(new_n289));
  oabi12aa1n02x5               g194(.a(new_n188), .b(new_n289), .c(new_n201), .out0(new_n290));
  and002aa1n02x5               g195(.a(new_n281), .b(new_n280), .o(new_n291));
  nano22aa1n02x4               g196(.a(new_n252), .b(new_n291), .c(new_n269), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n126), .d(new_n202), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n294));
  aobi12aa1n02x5               g199(.a(new_n294), .b(new_n275), .c(new_n291), .out0(new_n295));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n297), .b(new_n296), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n295), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n296), .clkout(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[27] ), .b(\a[28] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n247), .b(new_n256), .c(new_n262), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n226), .c(new_n224), .d(new_n218), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n291), .clkout(new_n304));
  aoai13aa1n02x5               g209(.a(new_n294), .b(new_n304), .c(new_n303), .d(new_n274), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n297), .b(new_n305), .c(new_n190), .d(new_n292), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n301), .b(new_n306), .c(new_n300), .o1(new_n307));
  aoi022aa1n02x5               g212(.a(new_n293), .b(new_n295), .c(\b[26] ), .d(\a[27] ), .o1(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n300), .c(new_n301), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[28] ));
  nano22aa1n02x4               g215(.a(new_n301), .b(new_n300), .c(new_n297), .out0(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n305), .c(new_n190), .d(new_n292), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[28] ), .b(\a[29] ), .out0(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  160nm_ficinv00aa1n08x5       g220(.clk(new_n311), .clkout(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n293), .c(new_n295), .o1(new_n317));
  nano22aa1n02x4               g222(.a(new_n317), .b(new_n313), .c(new_n314), .out0(new_n318));
  norp02aa1n02x5               g223(.a(new_n315), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g225(.a(new_n298), .b(new_n314), .c(new_n301), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n305), .c(new_n190), .d(new_n292), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n313), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .out0(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  160nm_ficinv00aa1n08x5       g230(.clk(new_n321), .clkout(new_n326));
  aoi012aa1n02x5               g231(.a(new_n326), .b(new_n293), .c(new_n295), .o1(new_n327));
  nano22aa1n02x4               g232(.a(new_n327), .b(new_n323), .c(new_n324), .out0(new_n328));
  norp02aa1n02x5               g233(.a(new_n325), .b(new_n328), .o1(\s[30] ));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  norb03aa1n02x5               g235(.a(new_n311), .b(new_n324), .c(new_n314), .out0(new_n331));
  160nm_ficinv00aa1n08x5       g236(.clk(new_n331), .clkout(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n293), .c(new_n295), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n334));
  nano22aa1n02x4               g239(.a(new_n333), .b(new_n330), .c(new_n334), .out0(new_n335));
  aoai13aa1n02x5               g240(.a(new_n331), .b(new_n305), .c(new_n190), .d(new_n292), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n330), .b(new_n336), .c(new_n334), .o1(new_n337));
  norp02aa1n02x5               g242(.a(new_n337), .b(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g243(.a(new_n101), .b(new_n104), .c(new_n109), .out0(\s[3] ));
  oaoi03aa1n02x5               g244(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g246(.a(new_n197), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g247(.a(\b[4] ), .b(\a[5] ), .o1(new_n343));
  aoi012aa1n02x5               g248(.a(new_n122), .b(new_n197), .c(new_n343), .o1(new_n344));
  xorb03aa1n02x5               g249(.a(new_n344), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  oaoi03aa1n02x5               g250(.a(\a[6] ), .b(\b[5] ), .c(new_n344), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g252(.a(new_n114), .b(new_n346), .c(new_n115), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


