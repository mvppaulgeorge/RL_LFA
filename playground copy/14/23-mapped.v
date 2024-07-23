// Benchmark "adder" written by ABC on Thu Jul 11 11:56:12 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n299, new_n302, new_n304, new_n306;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[5] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[4] ), .clkout(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  160nm_fiao0012aa1n02p5x5     g027(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n123));
  aoib12aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n114), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n97), .b(new_n98), .c(new_n118), .d(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xorc02aa1n02x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  norp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  oaoi03aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n130), .b(new_n131), .c(new_n125), .d(new_n127), .o1(new_n132));
  aoi112aa1n02x5               g037(.a(new_n130), .b(new_n131), .c(new_n125), .d(new_n127), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(\s[11] ));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  nona22aa1n02x4               g042(.a(new_n132), .b(new_n137), .c(new_n128), .out0(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n137), .clkout(new_n139));
  oaoi13aa1n02x5               g044(.a(new_n139), .b(new_n132), .c(\a[11] ), .d(\b[10] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n138), .b(new_n140), .out0(\s[12] ));
  nano23aa1n02x4               g046(.a(new_n128), .b(new_n135), .c(new_n136), .d(new_n129), .out0(new_n142));
  xorc02aa1n02x5               g047(.a(\a[9] ), .b(\b[8] ), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n142), .b(new_n127), .c(new_n143), .o1(new_n144));
  160nm_fiao0012aa1n02p5x5     g049(.a(new_n135), .b(new_n128), .c(new_n136), .o(new_n145));
  aoi012aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n131), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n144), .c(new_n118), .d(new_n124), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g053(.a(\a[13] ), .b(\b[12] ), .o(new_n149));
  xorc02aa1n02x5               g054(.a(\a[13] ), .b(\b[12] ), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n147), .b(new_n150), .o1(new_n151));
  xorc02aa1n02x5               g056(.a(\a[14] ), .b(\b[13] ), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n151), .c(new_n149), .out0(\s[14] ));
  norp02aa1n02x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  and002aa1n02x5               g061(.a(new_n152), .b(new_n150), .o(new_n157));
  oao003aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n149), .carry(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  aoai13aa1n02x5               g064(.a(new_n156), .b(new_n159), .c(new_n147), .d(new_n157), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n156), .b(new_n159), .c(new_n147), .d(new_n157), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  nona22aa1n02x4               g070(.a(new_n160), .b(new_n165), .c(new_n154), .out0(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n165), .clkout(new_n167));
  oaoi13aa1n02x5               g072(.a(new_n167), .b(new_n160), .c(\a[15] ), .d(\b[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n166), .b(new_n168), .out0(\s[16] ));
  aoi012aa1n02x5               g074(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n170));
  oaib12aa1n02x5               g075(.a(new_n170), .b(new_n114), .c(new_n122), .out0(new_n171));
  nano23aa1n02x4               g076(.a(new_n154), .b(new_n163), .c(new_n164), .d(new_n155), .out0(new_n172));
  nanp03aa1n02x5               g077(.a(new_n172), .b(new_n150), .c(new_n152), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n144), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n171), .c(new_n109), .d(new_n117), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n163), .b(new_n154), .c(new_n164), .o1(new_n176));
  oaib12aa1n02x5               g081(.a(new_n176), .b(new_n158), .c(new_n172), .out0(new_n177));
  oab012aa1n02x4               g082(.a(new_n177), .b(new_n146), .c(new_n173), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n175), .b(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[18] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d04x5               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n183), .b(new_n182), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n188));
  norp02aa1n02x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n02x4               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n198), .clkout(new_n200));
  oaoi13aa1n02x5               g105(.a(new_n200), .b(new_n192), .c(\a[19] ), .d(\b[18] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n199), .b(new_n201), .out0(\s[20] ));
  nano23aa1n02x4               g107(.a(new_n189), .b(new_n196), .c(new_n197), .d(new_n190), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n186), .b(new_n203), .o1(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n205), .b(new_n181), .c(\b[17] ), .out0(new_n206));
  nona23aa1n02x4               g111(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n196), .b(new_n189), .c(new_n197), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n206), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n204), .c(new_n175), .d(new_n178), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[22] ), .b(\b[21] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n213), .c(new_n211), .d(new_n214), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[21] ), .clkout(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(\a[22] ), .clkout(new_n220));
  xroi22aa1d04x5               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  nanp03aa1n02x5               g126(.a(new_n221), .b(new_n186), .c(new_n203), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\b[21] ), .clkout(new_n223));
  oaoi03aa1n02x5               g128(.a(new_n220), .b(new_n223), .c(new_n213), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n209), .c(new_n221), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n222), .c(new_n175), .d(new_n178), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g133(.a(\b[22] ), .b(\a[23] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(\s[24] ));
  and002aa1n02x5               g139(.a(new_n231), .b(new_n230), .o(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  nano32aa1n02x4               g141(.a(new_n236), .b(new_n221), .c(new_n186), .d(new_n203), .out0(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n208), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n221), .b(new_n238), .c(new_n203), .d(new_n188), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(\a[24] ), .c(\b[23] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n236), .c(new_n239), .d(new_n224), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n179), .c(new_n237), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[24] ), .b(\a[25] ), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  xnrc02aa1n02x5               g150(.a(new_n243), .b(new_n245), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n245), .b(new_n242), .c(new_n179), .d(new_n237), .o1(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[25] ), .b(\a[26] ), .out0(new_n250));
  nanp03aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n250), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n248), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(\s[26] ));
  norp02aa1n02x5               g158(.a(new_n250), .b(new_n244), .o1(new_n254));
  nano22aa1n02x4               g159(.a(new_n222), .b(new_n235), .c(new_n254), .out0(new_n255));
  nanp02aa1n02x5               g160(.a(new_n179), .b(new_n255), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .c(new_n248), .carry(new_n257));
  aobi12aa1n02x5               g162(.a(new_n257), .b(new_n242), .c(new_n254), .out0(new_n258));
  xorc02aa1n02x5               g163(.a(\a[27] ), .b(\b[26] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .out0(\s[27] ));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n261), .clkout(new_n262));
  aobi12aa1n02x5               g167(.a(new_n259), .b(new_n258), .c(new_n256), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  aobi12aa1n02x5               g170(.a(new_n255), .b(new_n175), .c(new_n178), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n235), .b(new_n225), .c(new_n209), .d(new_n221), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n254), .clkout(new_n268));
  aoai13aa1n02x5               g173(.a(new_n257), .b(new_n268), .c(new_n267), .d(new_n241), .o1(new_n269));
  oai012aa1n02x5               g174(.a(new_n259), .b(new_n269), .c(new_n266), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n264), .b(new_n270), .c(new_n262), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n265), .o1(\s[28] ));
  norb02aa1n02x5               g177(.a(new_n259), .b(new_n264), .out0(new_n273));
  oai012aa1n02x5               g178(.a(new_n273), .b(new_n269), .c(new_n266), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[28] ), .b(\a[29] ), .out0(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n274), .c(new_n275), .o1(new_n277));
  aobi12aa1n02x5               g182(.a(new_n273), .b(new_n258), .c(new_n256), .out0(new_n278));
  nano22aa1n02x4               g183(.a(new_n278), .b(new_n275), .c(new_n276), .out0(new_n279));
  norp02aa1n02x5               g184(.a(new_n277), .b(new_n279), .o1(\s[29] ));
  xorb03aa1n02x5               g185(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g186(.a(new_n259), .b(new_n276), .c(new_n264), .out0(new_n282));
  oai012aa1n02x5               g187(.a(new_n282), .b(new_n269), .c(new_n266), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .c(new_n275), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[29] ), .b(\a[30] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n02x5               g191(.a(new_n282), .b(new_n258), .c(new_n256), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[30] ));
  norb02aa1n02x5               g194(.a(new_n282), .b(new_n285), .out0(new_n290));
  aobi12aa1n02x5               g195(.a(new_n290), .b(new_n258), .c(new_n256), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .c(new_n284), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[30] ), .b(\a[31] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n290), .b(new_n269), .c(new_n266), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n294), .o1(\s[31] ));
  xnrb03aa1n02x5               g202(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g203(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n299));
  xorb03aa1n02x5               g204(.a(new_n299), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g205(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g206(.a(new_n119), .b(new_n120), .c(new_n109), .o1(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g208(.a(\a[6] ), .b(\b[5] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g210(.a(new_n112), .b(new_n304), .c(new_n113), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g212(.a(new_n143), .b(new_n118), .c(new_n124), .out0(\s[9] ));
endmodule

