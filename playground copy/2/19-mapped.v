// Benchmark "adder" written by ABC on Wed Jul 10 17:24:02 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n309, new_n310, new_n312, new_n313,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n02x5               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n114), .b(new_n115), .c(new_n109), .o1(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[5] ), .clkout(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  oaib12aa1n02x5               g023(.a(new_n118), .b(new_n117), .c(\a[6] ), .out0(new_n119));
  aoi012aa1n02x5               g024(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .o1(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n123));
  xorc02aa1n02x5               g028(.a(\a[10] ), .b(\b[9] ), .out0(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n97), .out0(\s[10] ));
  aobi12aa1n02x5               g030(.a(new_n124), .b(new_n123), .c(new_n97), .out0(new_n126));
  norp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n130));
  oai012aa1n02x5               g035(.a(new_n129), .b(new_n126), .c(new_n130), .o1(new_n131));
  norp03aa1n02x5               g036(.a(new_n126), .b(new_n129), .c(new_n130), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(\s[11] ));
  norp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  nona22aa1n02x4               g041(.a(new_n131), .b(new_n136), .c(new_n127), .out0(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n127), .clkout(new_n138));
  aobi12aa1n02x5               g043(.a(new_n136), .b(new_n131), .c(new_n138), .out0(new_n139));
  norb02aa1n02x5               g044(.a(new_n137), .b(new_n139), .out0(\s[12] ));
  nano23aa1n02x4               g045(.a(new_n127), .b(new_n134), .c(new_n135), .d(new_n128), .out0(new_n141));
  and003aa1n02x5               g046(.a(new_n141), .b(new_n124), .c(new_n122), .o(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n143));
  aoi012aa1n02x5               g048(.a(new_n134), .b(new_n127), .c(new_n135), .o1(new_n144));
  aobi12aa1n02x5               g049(.a(new_n144), .b(new_n141), .c(new_n130), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n143), .b(new_n145), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n148), .b(new_n146), .c(new_n149), .o1(new_n150));
  xnrb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g056(.a(\b[14] ), .b(\a[15] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[14] ), .b(\a[15] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n148), .b(new_n156), .c(new_n157), .d(new_n149), .out0(new_n158));
  oa0012aa1n02x5               g063(.a(new_n157), .b(new_n156), .c(new_n148), .o(new_n159));
  aoai13aa1n02x5               g064(.a(new_n155), .b(new_n159), .c(new_n146), .d(new_n158), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n155), .b(new_n159), .c(new_n146), .d(new_n158), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  oai112aa1n02x5               g070(.a(new_n160), .b(new_n165), .c(\b[14] ), .d(\a[15] ), .o1(new_n166));
  oaoi13aa1n02x5               g071(.a(new_n165), .b(new_n160), .c(\a[15] ), .d(\b[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[16] ));
  nona22aa1n02x4               g073(.a(new_n158), .b(new_n165), .c(new_n154), .out0(new_n169));
  nano32aa1n02x4               g074(.a(new_n169), .b(new_n141), .c(new_n124), .d(new_n122), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n163), .clkout(new_n172));
  nona22aa1n02x4               g077(.a(new_n159), .b(new_n165), .c(new_n154), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n152), .b(new_n164), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n141), .b(new_n130), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n169), .b(new_n175), .c(new_n144), .o1(new_n176));
  nano32aa1n02x4               g081(.a(new_n176), .b(new_n174), .c(new_n173), .d(new_n172), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n171), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d04x5               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n181), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[18] ), .b(\b[17] ), .c(new_n186), .o1(new_n187));
  norp02aa1n02x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  nona22aa1n02x4               g102(.a(new_n191), .b(new_n197), .c(new_n188), .out0(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n188), .clkout(new_n199));
  aobi12aa1n02x5               g104(.a(new_n197), .b(new_n191), .c(new_n199), .out0(new_n200));
  norb02aa1n02x5               g105(.a(new_n198), .b(new_n200), .out0(\s[20] ));
  nano23aa1n02x4               g106(.a(new_n188), .b(new_n195), .c(new_n196), .d(new_n189), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n185), .b(new_n202), .o1(new_n203));
  oai022aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n02x5               g109(.a(new_n204), .b(new_n180), .c(\b[17] ), .out0(new_n205));
  nona23aa1n02x4               g110(.a(new_n196), .b(new_n189), .c(new_n188), .d(new_n195), .out0(new_n206));
  aoi012aa1n02x5               g111(.a(new_n195), .b(new_n188), .c(new_n196), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n177), .d(new_n171), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g122(.clk(\a[21] ), .clkout(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[22] ), .clkout(new_n219));
  xroi22aa1d04x5               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  nanp03aa1n02x5               g125(.a(new_n220), .b(new_n185), .c(new_n202), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(\b[21] ), .clkout(new_n222));
  oao003aa1n02x5               g127(.a(new_n219), .b(new_n222), .c(new_n212), .carry(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n208), .c(new_n220), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n221), .c(new_n177), .d(new_n171), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[24] ), .b(\b[23] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[23] ), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[24] ), .clkout(new_n234));
  xroi22aa1d04x5               g139(.a(new_n233), .b(\b[22] ), .c(new_n234), .d(\b[23] ), .out0(new_n235));
  nano22aa1n02x4               g140(.a(new_n203), .b(new_n220), .c(new_n235), .out0(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n207), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n220), .b(new_n237), .c(new_n202), .d(new_n187), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n223), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n235), .clkout(new_n240));
  aoi112aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n241));
  aoib12aa1n02x5               g146(.a(new_n241), .b(new_n234), .c(\b[23] ), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n239), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[25] ), .b(\b[24] ), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n178), .d(new_n236), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n244), .b(new_n243), .c(new_n178), .d(new_n236), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[26] ), .b(\b[25] ), .out0(new_n249));
  nona22aa1n02x4               g154(.a(new_n245), .b(new_n249), .c(new_n248), .out0(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n248), .clkout(new_n251));
  aobi12aa1n02x5               g156(.a(new_n249), .b(new_n245), .c(new_n251), .out0(new_n252));
  norb02aa1n02x5               g157(.a(new_n250), .b(new_n252), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g158(.clk(\a[25] ), .clkout(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(\a[26] ), .clkout(new_n255));
  xroi22aa1d04x5               g160(.a(new_n254), .b(\b[24] ), .c(new_n255), .d(\b[25] ), .out0(new_n256));
  nano32aa1n02x4               g161(.a(new_n203), .b(new_n256), .c(new_n220), .d(new_n235), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n178), .b(new_n257), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n251), .carry(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoi012aa1n02x5               g165(.a(new_n260), .b(new_n243), .c(new_n256), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n258), .c(new_n261), .out0(\s[27] ));
  norp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n262), .clkout(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n258), .c(new_n261), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n267), .b(new_n265), .c(new_n268), .out0(new_n269));
  aobi12aa1n02x5               g174(.a(new_n257), .b(new_n177), .c(new_n171), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n235), .b(new_n223), .c(new_n208), .d(new_n220), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n256), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n259), .b(new_n272), .c(new_n271), .d(new_n242), .o1(new_n273));
  oai012aa1n02x5               g178(.a(new_n262), .b(new_n273), .c(new_n270), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n268), .b(new_n274), .c(new_n265), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n269), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n262), .b(new_n268), .out0(new_n277));
  oai012aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n270), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n277), .clkout(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n258), .c(new_n261), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n279), .c(new_n280), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n281), .b(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g191(.a(new_n262), .b(new_n280), .c(new_n268), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n273), .c(new_n270), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n287), .clkout(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n258), .c(new_n261), .o1(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n289), .c(new_n290), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n291), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n287), .b(new_n290), .out0(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n296), .clkout(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n258), .c(new_n261), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n298), .b(new_n299), .c(new_n300), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n296), .b(new_n273), .c(new_n270), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n300), .b(new_n302), .c(new_n299), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n303), .b(new_n301), .o1(\s[31] ));
  xnrb03aa1n02x5               g209(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g213(.a(new_n115), .b(new_n108), .out0(new_n309));
  oai012aa1n02x5               g214(.a(new_n309), .b(\b[4] ), .c(\a[5] ), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g216(.a(new_n109), .b(new_n310), .out0(new_n312));
  oaib12aa1n02x5               g217(.a(new_n312), .b(\a[6] ), .c(new_n117), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g219(.a(new_n110), .b(new_n111), .out0(new_n315));
  160nm_ficinv00aa1n08x5       g220(.clk(new_n112), .clkout(new_n316));
  nanb02aa1n02x5               g221(.a(new_n112), .b(new_n113), .out0(new_n317));
  nanb02aa1n02x5               g222(.a(new_n317), .b(new_n313), .out0(new_n318));
  aoi012aa1n02x5               g223(.a(new_n315), .b(new_n318), .c(new_n316), .o1(new_n319));
  nanp03aa1n02x5               g224(.a(new_n318), .b(new_n315), .c(new_n316), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(\s[8] ));
  aoi112aa1n02x5               g226(.a(new_n122), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n123), .b(new_n322), .out0(\s[9] ));
endmodule


