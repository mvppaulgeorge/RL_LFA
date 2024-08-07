// Benchmark "adder" written by ABC on Thu Jul 11 12:24:43 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[4] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[3] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanb02aa1n02x5               g012(.a(new_n106), .b(new_n107), .out0(new_n108));
  oaoi03aa1n02x5               g013(.a(new_n101), .b(new_n102), .c(new_n106), .o1(new_n109));
  oai013aa1n02x4               g014(.a(new_n109), .b(new_n100), .c(new_n108), .d(new_n105), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .out0(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .out0(new_n112));
  norp02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  norp03aa1n02x5               g022(.a(new_n117), .b(new_n112), .c(new_n111), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[8] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[7] ), .clkout(new_n121));
  norp02aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n123));
  oai013aa1n02x4               g028(.a(new_n123), .b(new_n112), .c(new_n111), .d(new_n119), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n110), .c(new_n118), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g032(.a(new_n110), .b(new_n118), .o1(new_n128));
  norp03aa1n02x5               g033(.a(new_n111), .b(new_n112), .c(new_n119), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n123), .b(new_n129), .out0(new_n130));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n132), .b(new_n131), .c(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[8] ), .b(\a[9] ), .o1(new_n135));
  nona23aa1n02x4               g040(.a(new_n133), .b(new_n135), .c(new_n131), .d(new_n132), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n134), .b(new_n136), .c(new_n128), .d(new_n130), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(\a[12] ), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n141), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(new_n139), .out0(\s[12] ));
  norp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n145), .b(new_n141), .c(new_n140), .d(new_n144), .out0(new_n146));
  160nm_fiao0012aa1n02p5x5     g051(.a(new_n144), .b(new_n140), .c(new_n145), .o(new_n147));
  oabi12aa1n02x5               g052(.a(new_n147), .b(new_n146), .c(new_n134), .out0(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  nano23aa1n02x4               g054(.a(new_n131), .b(new_n132), .c(new_n133), .d(new_n135), .out0(new_n150));
  nano23aa1n02x4               g055(.a(new_n140), .b(new_n144), .c(new_n145), .d(new_n141), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n149), .b(new_n152), .c(new_n128), .d(new_n130), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g059(.clk(\a[14] ), .clkout(new_n155));
  norp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n153), .c(new_n157), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(new_n155), .out0(\s[14] ));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n162));
  aoi012aa1n02x5               g067(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n163));
  nano23aa1n02x4               g068(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n164));
  aobi12aa1n02x5               g069(.a(new_n163), .b(new_n148), .c(new_n164), .out0(new_n165));
  oai013aa1n02x4               g070(.a(new_n165), .b(new_n125), .c(new_n152), .d(new_n162), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n168), .b(new_n172), .c(new_n166), .d(new_n169), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n02x4               g080(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n176));
  nano22aa1n02x4               g081(.a(new_n152), .b(new_n164), .c(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n168), .b(new_n169), .out0(new_n179));
  norp03aa1n02x5               g084(.a(new_n162), .b(new_n179), .c(new_n172), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n170), .b(new_n168), .c(new_n171), .o1(new_n181));
  oai013aa1n02x4               g086(.a(new_n181), .b(new_n163), .c(new_n179), .d(new_n172), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n148), .c(new_n180), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n178), .c(new_n183), .out0(\s[17] ));
  nanp02aa1n02x5               g090(.a(new_n178), .b(new_n183), .o1(new_n186));
  norp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n187), .b(new_n186), .c(new_n184), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\b[17] ), .clkout(new_n190));
  nanp02aa1n02x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n188), .b(new_n192), .c(new_n191), .out0(\s[18] ));
  aob012aa1n02x5               g098(.a(new_n191), .b(new_n187), .c(new_n192), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n184), .clkout(new_n196));
  nano22aa1n02x4               g101(.a(new_n196), .b(new_n191), .c(new_n192), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  aoai13aa1n02x5               g103(.a(new_n195), .b(new_n198), .c(new_n178), .d(new_n183), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n206), .clkout(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n203), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n203), .b(new_n202), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n199), .b(new_n209), .o1(new_n210));
  nona22aa1n02x4               g115(.a(new_n210), .b(new_n207), .c(new_n202), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n211), .b(new_n208), .o1(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n202), .b(new_n204), .c(new_n205), .d(new_n203), .out0(new_n213));
  aoi012aa1n02x5               g118(.a(new_n204), .b(new_n202), .c(new_n205), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoi012aa1n02x5               g120(.a(new_n215), .b(new_n213), .c(new_n194), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n197), .b(new_n213), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n217), .c(new_n178), .d(new_n183), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norp02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[21] ), .b(\a[22] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n218), .b(new_n222), .o1(new_n228));
  nona22aa1n02x4               g133(.a(new_n228), .b(new_n226), .c(new_n220), .out0(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(\s[22] ));
  nano23aa1n02x4               g135(.a(new_n220), .b(new_n223), .c(new_n224), .d(new_n221), .out0(new_n231));
  nano22aa1n02x4               g136(.a(new_n198), .b(new_n213), .c(new_n231), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n216), .clkout(new_n234));
  aoi012aa1n02x5               g139(.a(new_n223), .b(new_n220), .c(new_n224), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n234), .c(new_n231), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n178), .d(new_n183), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  norp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  nanb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n240), .c(new_n238), .d(new_n242), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n238), .b(new_n242), .o1(new_n247));
  nona22aa1n02x4               g152(.a(new_n247), .b(new_n245), .c(new_n240), .out0(new_n248));
  nanp02aa1n02x5               g153(.a(new_n248), .b(new_n246), .o1(\s[24] ));
  nanp03aa1n02x5               g154(.a(new_n194), .b(new_n209), .c(new_n206), .o1(new_n250));
  nano23aa1n02x4               g155(.a(new_n240), .b(new_n243), .c(new_n244), .d(new_n241), .out0(new_n251));
  nanp02aa1n02x5               g156(.a(new_n251), .b(new_n231), .o1(new_n252));
  160nm_fiao0012aa1n02p5x5     g157(.a(new_n243), .b(new_n240), .c(new_n244), .o(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n251), .c(new_n236), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n214), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  nano32aa1n02x4               g161(.a(new_n198), .b(new_n251), .c(new_n213), .d(new_n231), .out0(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  aoai13aa1n02x5               g163(.a(new_n256), .b(new_n258), .c(new_n178), .d(new_n183), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n259), .b(new_n262), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n263), .c(new_n261), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n266), .b(new_n264), .o1(\s[26] ));
  nano32aa1n02x4               g172(.a(new_n245), .b(new_n242), .c(new_n225), .d(new_n222), .out0(new_n268));
  norb02aa1n02x5               g173(.a(new_n262), .b(new_n263), .out0(new_n269));
  nanb03aa1n02x5               g174(.a(new_n217), .b(new_n269), .c(new_n268), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(\a[26] ), .clkout(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(\b[25] ), .clkout(new_n272));
  oaoi03aa1n02x5               g177(.a(new_n271), .b(new_n272), .c(new_n261), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(new_n255), .c(new_n269), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n270), .c(new_n178), .d(new_n183), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .d(new_n279), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n217), .b(new_n268), .c(new_n269), .out0(new_n282));
  nanp02aa1n02x5               g187(.a(new_n255), .b(new_n269), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n283), .b(new_n273), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n186), .d(new_n282), .o1(new_n285));
  nona22aa1n02x4               g190(.a(new_n285), .b(new_n280), .c(new_n278), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n281), .b(new_n286), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g192(.clk(\a[28] ), .clkout(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(\b[27] ), .clkout(new_n289));
  oaoi03aa1n02x5               g194(.a(new_n288), .b(new_n289), .c(new_n278), .o1(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n290), .clkout(new_n291));
  norb02aa1n02x5               g196(.a(new_n279), .b(new_n280), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n284), .c(new_n186), .d(new_n282), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nona22aa1n02x4               g199(.a(new_n293), .b(new_n294), .c(new_n291), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n294), .b(new_n291), .c(new_n276), .d(new_n292), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n296), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oaoi03aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .o1(new_n299));
  norb03aa1n02x5               g204(.a(new_n279), .b(new_n294), .c(new_n280), .out0(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n276), .d(new_n300), .o1(new_n302));
  aoai13aa1n02x5               g207(.a(new_n300), .b(new_n284), .c(new_n186), .d(new_n282), .o1(new_n303));
  nona22aa1n02x4               g208(.a(new_n303), .b(new_n301), .c(new_n299), .out0(new_n304));
  nanp02aa1n02x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  norb02aa1n02x5               g210(.a(new_n300), .b(new_n301), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n284), .c(new_n186), .d(new_n282), .o1(new_n307));
  nanb02aa1n02x5               g212(.a(new_n301), .b(new_n299), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n308), .b(\b[29] ), .c(\a[30] ), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nona22aa1n02x4               g215(.a(new_n307), .b(new_n309), .c(new_n310), .out0(new_n311));
  aoai13aa1n02x5               g216(.a(new_n310), .b(new_n309), .c(new_n276), .d(new_n306), .o1(new_n312));
  nanp02aa1n02x5               g217(.a(new_n312), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g222(.a(new_n115), .b(new_n110), .c(new_n116), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g224(.a(new_n119), .b(new_n117), .c(new_n110), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g226(.a(new_n122), .b(new_n320), .c(new_n112), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(new_n120), .out0(\s[8] ));
  xnrb03aa1n02x5               g228(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


