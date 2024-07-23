// Benchmark "adder" written by ABC on Wed Jul 10 16:40:04 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n315, new_n317;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[4] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  aboi22aa1n03x5               g014(.a(\b[3] ), .b(new_n100), .c(new_n101), .d(new_n102), .out0(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(new_n105), .o1(new_n111));
  oaib12aa1n02x5               g016(.a(new_n111), .b(new_n100), .c(\b[3] ), .out0(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n02x4               g021(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[6] ), .b(\b[5] ), .out0(new_n118));
  xorc02aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nanp03aa1n02x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n121));
  orn002aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .o(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aobi12aa1n02x5               g028(.a(new_n121), .b(new_n117), .c(new_n123), .out0(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n112), .c(new_n120), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n98), .b(new_n99), .c(new_n125), .d(new_n126), .o1(new_n127));
  and002aa1n02x5               g032(.a(\b[3] ), .b(\a[4] ), .o(new_n128));
  oaoi13aa1n02x5               g033(.a(new_n128), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n129));
  nona23aa1n02x4               g034(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n130));
  nano22aa1n02x4               g035(.a(new_n130), .b(new_n118), .c(new_n119), .out0(new_n131));
  oaib12aa1n02x5               g036(.a(new_n121), .b(new_n130), .c(new_n123), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n126), .b(new_n132), .c(new_n131), .d(new_n129), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n133), .b(new_n99), .c(new_n98), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n127), .b(new_n134), .o1(\s[10] ));
  160nm_ficinv00aa1n08x5       g040(.clk(\a[10] ), .clkout(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(\b[9] ), .clkout(new_n137));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  oai112aa1n02x5               g045(.a(new_n134), .b(new_n140), .c(new_n137), .d(new_n136), .o1(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n140), .b(new_n134), .c(new_n136), .d(new_n137), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n138), .clkout(new_n144));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n141), .c(new_n144), .out0(\s[12] ));
  xnrc02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .out0(new_n149));
  nano23aa1n02x4               g054(.a(new_n138), .b(new_n145), .c(new_n146), .d(new_n139), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n150), .b(new_n97), .c(new_n126), .o1(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n132), .c(new_n131), .d(new_n129), .o1(new_n153));
  nona23aa1n02x4               g058(.a(new_n146), .b(new_n139), .c(new_n138), .d(new_n145), .out0(new_n154));
  oai012aa1n02x5               g059(.a(new_n146), .b(new_n145), .c(new_n138), .o1(new_n155));
  oaoi03aa1n02x5               g060(.a(new_n136), .b(new_n137), .c(new_n99), .o1(new_n156));
  oai012aa1n02x5               g061(.a(new_n155), .b(new_n154), .c(new_n156), .o1(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  xobna2aa1n03x5               g063(.a(new_n149), .b(new_n153), .c(new_n158), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n149), .c(new_n153), .d(new_n158), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  norp02aa1n02x5               g068(.a(new_n163), .b(new_n149), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  oao003aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .carry(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n153), .d(new_n158), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoi112aa1n02x5               g078(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n169), .b(new_n171), .c(new_n172), .d(new_n170), .out0(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n163), .c(new_n149), .out0(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n151), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n171), .b(new_n169), .c(new_n172), .o1(new_n181));
  oaib12aa1n02x5               g086(.a(new_n181), .b(new_n166), .c(new_n177), .out0(new_n182));
  aoi013aa1n02x4               g087(.a(new_n182), .b(new_n157), .c(new_n164), .d(new_n177), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n188), .b(new_n187), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x4               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n194), .clkout(new_n205));
  aobi12aa1n02x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n191), .b(new_n208), .o1(new_n209));
  oai022aa1n02x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n02x5               g115(.a(new_n210), .b(new_n186), .c(\b[17] ), .out0(new_n211));
  nona23aa1n02x4               g116(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n212));
  aoi012aa1n02x5               g117(.a(new_n201), .b(new_n194), .c(new_n202), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n212), .c(new_n211), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n209), .c(new_n180), .d(new_n183), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n191), .c(new_n208), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n226), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n180), .d(new_n183), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g143(.clk(\a[23] ), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\a[24] ), .clkout(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[22] ), .c(new_n240), .d(\b[23] ), .out0(new_n241));
  nano22aa1n02x4               g146(.a(new_n209), .b(new_n226), .c(new_n241), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n213), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n226), .b(new_n243), .c(new_n208), .d(new_n193), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n229), .clkout(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n241), .clkout(new_n246));
  aoi112aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n247));
  aoib12aa1n02x5               g152(.a(new_n247), .b(new_n240), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n245), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n249), .c(new_n184), .d(new_n242), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n184), .d(new_n242), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  nona22aa1n02x4               g160(.a(new_n251), .b(new_n255), .c(new_n254), .out0(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n254), .clkout(new_n257));
  aobi12aa1n02x5               g162(.a(new_n255), .b(new_n251), .c(new_n257), .out0(new_n258));
  norb02aa1n02x5               g163(.a(new_n256), .b(new_n258), .out0(\s[26] ));
  oabi12aa1n02x5               g164(.a(new_n182), .b(new_n158), .c(new_n178), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(\a[25] ), .clkout(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(\a[26] ), .clkout(new_n262));
  xroi22aa1d04x5               g167(.a(new_n261), .b(\b[24] ), .c(new_n262), .d(\b[25] ), .out0(new_n263));
  nano32aa1n02x4               g168(.a(new_n209), .b(new_n263), .c(new_n226), .d(new_n241), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n260), .c(new_n125), .d(new_n179), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n266));
  aobi12aa1n02x5               g171(.a(new_n266), .b(new_n249), .c(new_n263), .out0(new_n267));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  norb02aa1n02x5               g174(.a(new_n269), .b(new_n268), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n267), .c(new_n265), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n268), .clkout(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aobi12aa1n02x5               g178(.a(new_n264), .b(new_n180), .c(new_n183), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n241), .b(new_n229), .c(new_n214), .d(new_n226), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n263), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n266), .b(new_n276), .c(new_n275), .d(new_n248), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n269), .b(new_n277), .c(new_n274), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n273), .b(new_n278), .c(new_n272), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n269), .b(new_n267), .c(new_n265), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n272), .c(new_n273), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n279), .b(new_n281), .o1(\s[28] ));
  nano22aa1n02x4               g187(.a(new_n273), .b(new_n272), .c(new_n269), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n277), .c(new_n274), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n283), .b(new_n267), .c(new_n265), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n270), .b(new_n286), .c(new_n273), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n277), .c(new_n274), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x5               g201(.a(new_n292), .b(new_n267), .c(new_n265), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  norb03aa1n02x5               g205(.a(new_n283), .b(new_n295), .c(new_n286), .out0(new_n301));
  aobi12aa1n02x5               g206(.a(new_n301), .b(new_n267), .c(new_n265), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n303));
  nano22aa1n02x4               g208(.a(new_n302), .b(new_n300), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n301), .b(new_n277), .c(new_n274), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n300), .b(new_n305), .c(new_n303), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnbna2aa1n03x5               g212(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n129), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g216(.a(\a[5] ), .b(\b[4] ), .c(new_n112), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanp02aa1n02x5               g218(.a(\b[5] ), .b(\a[6] ), .o1(new_n314));
  oaib12aa1n02x5               g219(.a(new_n314), .b(new_n312), .c(new_n118), .out0(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g221(.a(\a[7] ), .b(\b[6] ), .c(new_n315), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


