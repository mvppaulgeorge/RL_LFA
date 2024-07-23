// Benchmark "adder" written by ABC on Thu Jul 11 11:25:54 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n309, new_n310, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nano23aa1n02x4               g012(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(new_n108), .clkout(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(\a[2] ), .clkout(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\b[1] ), .clkout(new_n111));
  nanp02aa1n02x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  oaoi03aa1n02x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  160nm_fiao0012aa1n02p5x5     g023(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n119));
  oabi12aa1n02x5               g024(.a(new_n119), .b(new_n118), .c(new_n113), .out0(new_n120));
  nona22aa1n02x4               g025(.a(new_n120), .b(new_n109), .c(new_n103), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(new_n101), .b(new_n100), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\a[5] ), .clkout(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\b[4] ), .clkout(new_n124));
  aoai13aa1n02x5               g029(.a(new_n105), .b(new_n104), .c(new_n123), .d(new_n124), .o1(new_n125));
  oai122aa1n02x7               g030(.a(new_n122), .b(new_n103), .c(new_n125), .d(\b[7] ), .e(\a[8] ), .o1(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n126), .clkout(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n97), .b(new_n128), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n98), .b(new_n129), .c(new_n127), .d(new_n121), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  oai012aa1n02x5               g041(.a(new_n136), .b(new_n97), .c(new_n135), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n108), .b(new_n103), .out0(new_n138));
  nano23aa1n02x4               g043(.a(new_n135), .b(new_n97), .c(new_n128), .d(new_n136), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n126), .c(new_n138), .d(new_n120), .o1(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n134), .b(new_n140), .c(new_n137), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n132), .clkout(new_n142));
  nanp02aa1n02x5               g047(.a(new_n121), .b(new_n127), .o1(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n137), .clkout(new_n144));
  aoai13aa1n02x5               g049(.a(new_n134), .b(new_n144), .c(new_n143), .d(new_n139), .o1(new_n145));
  norp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  xobna2aa1n03x5               g053(.a(new_n148), .b(new_n145), .c(new_n142), .out0(\s[12] ));
  nona23aa1n02x4               g054(.a(new_n147), .b(new_n133), .c(new_n132), .d(new_n146), .out0(new_n150));
  norb02aa1n02x5               g055(.a(new_n139), .b(new_n150), .out0(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n126), .c(new_n138), .d(new_n120), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n146), .clkout(new_n153));
  nanp02aa1n02x5               g058(.a(new_n132), .b(new_n147), .o1(new_n154));
  oai112aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n150), .d(new_n137), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  nanp02aa1n02x5               g061(.a(new_n152), .b(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n02x4               g069(.a(new_n164), .b(new_n160), .c(new_n159), .d(new_n163), .out0(new_n165));
  oai012aa1n02x5               g070(.a(new_n164), .b(new_n163), .c(new_n159), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n152), .d(new_n156), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  xorc02aa1n02x5               g074(.a(\a[15] ), .b(\b[14] ), .out0(new_n170));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n132), .b(new_n146), .c(new_n147), .d(new_n133), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n171), .b(new_n170), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n176), .b(new_n165), .c(new_n175), .d(new_n139), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n126), .c(new_n138), .d(new_n120), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n165), .b(new_n170), .c(new_n171), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  oai022aa1n02x5               g085(.a(new_n176), .b(new_n166), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n180), .c(new_n155), .d(new_n179), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n187), .b(new_n186), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x4               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  orn002aa1n02x5               g108(.a(\a[19] ), .b(\b[18] ), .o(new_n204));
  aobi12aa1n02x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n190), .b(new_n207), .o1(new_n208));
  oai022aa1n02x5               g113(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n209));
  oaib12aa1n02x5               g114(.a(new_n209), .b(new_n185), .c(\b[17] ), .out0(new_n210));
  nona23aa1n02x4               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n212));
  oabi12aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n210), .out0(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n208), .c(new_n182), .d(new_n178), .o1(new_n215));
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
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n190), .c(new_n207), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oaoi03aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n213), .c(new_n225), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n226), .c(new_n182), .d(new_n178), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n225), .c(new_n190), .d(new_n207), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n225), .b(new_n212), .c(new_n207), .d(new_n192), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n243));
  oab012aa1n02x4               g148(.a(new_n243), .b(\a[24] ), .c(\b[23] ), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .d(new_n228), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n241), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n241), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x4               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n250), .clkout(new_n253));
  aobi12aa1n02x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  and002aa1n02x5               g160(.a(new_n251), .b(new_n246), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n226), .b(new_n239), .c(new_n256), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n183), .b(new_n257), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n245), .c(new_n256), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n258), .c(new_n260), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n261), .clkout(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n257), .clkout(new_n267));
  aoi012aa1n02x5               g172(.a(new_n267), .b(new_n182), .c(new_n178), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n239), .b(new_n229), .c(new_n213), .d(new_n225), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n256), .clkout(new_n270));
  aoai13aa1n02x5               g175(.a(new_n259), .b(new_n270), .c(new_n269), .d(new_n244), .o1(new_n271));
  oai012aa1n02x5               g176(.a(new_n262), .b(new_n271), .c(new_n268), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n266), .b(new_n272), .c(new_n265), .o1(new_n273));
  aoi022aa1n02x5               g178(.a(new_n258), .b(new_n260), .c(\b[26] ), .d(\a[27] ), .o1(new_n274));
  nano22aa1n02x4               g179(.a(new_n274), .b(new_n265), .c(new_n266), .out0(new_n275));
  norp02aa1n02x5               g180(.a(new_n273), .b(new_n275), .o1(\s[28] ));
  nano22aa1n02x4               g181(.a(new_n266), .b(new_n265), .c(new_n262), .out0(new_n277));
  oai012aa1n02x5               g182(.a(new_n277), .b(new_n271), .c(new_n268), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n02x5               g186(.a(new_n277), .b(new_n258), .c(new_n260), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  norp02aa1n02x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n263), .b(new_n280), .c(new_n266), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n271), .c(new_n268), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n286), .b(new_n258), .c(new_n260), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  norb03aa1n02x5               g199(.a(new_n277), .b(new_n289), .c(new_n280), .out0(new_n295));
  aobi12aa1n02x5               g200(.a(new_n295), .b(new_n258), .c(new_n260), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n297));
  nano22aa1n02x4               g202(.a(new_n296), .b(new_n294), .c(new_n297), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n295), .b(new_n271), .c(new_n268), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n294), .b(new_n299), .c(new_n297), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g210(.a(new_n123), .b(new_n124), .c(new_n120), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g212(.a(new_n101), .b(new_n102), .out0(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n120), .clkout(new_n309));
  oaoi13aa1n02x5               g214(.a(new_n308), .b(new_n125), .c(new_n309), .d(new_n109), .o1(new_n310));
  oai112aa1n02x5               g215(.a(new_n308), .b(new_n125), .c(new_n309), .d(new_n109), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n311), .b(new_n310), .out0(\s[7] ));
  norp02aa1n02x5               g217(.a(new_n310), .b(new_n101), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g219(.a(new_n129), .b(new_n127), .c(new_n121), .out0(\s[9] ));
endmodule


