// Benchmark "adder" written by ABC on Thu Jul 11 12:03:47 2024

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
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanb02aa1n02x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\a[3] ), .clkout(new_n106));
  nanb02aa1n02x5               g011(.a(\b[2] ), .b(new_n106), .out0(new_n107));
  oao003aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1n02x4               g013(.a(new_n108), .b(new_n102), .c(new_n101), .d(new_n105), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .out0(new_n111));
  norp02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n116), .b(new_n111), .c(new_n110), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n119));
  norp03aa1n02x5               g024(.a(new_n110), .b(new_n111), .c(new_n119), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\a[8] ), .clkout(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\b[7] ), .clkout(new_n122));
  norp02aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n120), .out0(new_n125));
  nanp02aa1n02x5               g030(.a(new_n118), .b(new_n125), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n130), .b(new_n97), .c(new_n131), .o1(new_n132));
  nona23aa1n02x4               g037(.a(new_n131), .b(new_n127), .c(new_n97), .d(new_n130), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(new_n118), .d(new_n125), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(\a[12] ), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n137), .b(new_n134), .c(new_n138), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[11] ), .c(new_n136), .out0(\s[12] ));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1n02x4               g047(.a(new_n142), .b(new_n138), .c(new_n137), .d(new_n141), .out0(new_n143));
  aoi012aa1n02x5               g048(.a(new_n141), .b(new_n137), .c(new_n142), .o1(new_n144));
  oai012aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n132), .o1(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  oai013aa1n02x4               g051(.a(new_n124), .b(new_n111), .c(new_n110), .d(new_n119), .o1(new_n147));
  norp02aa1n02x5               g052(.a(new_n143), .b(new_n133), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n147), .c(new_n109), .d(new_n117), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n146), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g056(.clk(\a[14] ), .clkout(new_n152));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n159), .b(new_n160), .c(new_n149), .d(new_n146), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  xorc02aa1n02x5               g068(.a(\a[15] ), .b(\b[14] ), .out0(new_n164));
  xnrc02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n161), .b(new_n164), .o1(new_n167));
  nona22aa1n02x4               g072(.a(new_n167), .b(new_n165), .c(new_n163), .out0(new_n168));
  nanp02aa1n02x5               g073(.a(new_n168), .b(new_n166), .o1(\s[16] ));
  nano23aa1n02x4               g074(.a(new_n97), .b(new_n130), .c(new_n131), .d(new_n127), .out0(new_n170));
  nano23aa1n02x4               g075(.a(new_n137), .b(new_n141), .c(new_n142), .d(new_n138), .out0(new_n171));
  nano23aa1n02x4               g076(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n172));
  xorc02aa1n02x5               g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  nanp03aa1n02x5               g078(.a(new_n172), .b(new_n164), .c(new_n173), .o1(new_n174));
  nano22aa1n02x4               g079(.a(new_n174), .b(new_n170), .c(new_n171), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n147), .c(new_n109), .d(new_n117), .o1(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .out0(new_n177));
  norp03aa1n02x5               g082(.a(new_n160), .b(new_n165), .c(new_n177), .o1(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n163), .clkout(new_n179));
  oao003aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n179), .carry(new_n180));
  oai013aa1n02x4               g085(.a(new_n180), .b(new_n165), .c(new_n177), .d(new_n159), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n181), .b(new_n145), .c(new_n178), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n176), .c(new_n182), .out0(\s[17] ));
  nanp02aa1n02x5               g089(.a(new_n178), .b(new_n148), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n182), .b(new_n185), .c(new_n118), .d(new_n125), .o1(new_n186));
  norp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n187), .b(new_n186), .c(new_n183), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\b[17] ), .clkout(new_n190));
  nanp02aa1n02x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n188), .b(new_n192), .c(new_n191), .out0(\s[18] ));
  aob012aa1n02x5               g098(.a(new_n191), .b(new_n187), .c(new_n192), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n183), .clkout(new_n196));
  nano22aa1n02x4               g101(.a(new_n196), .b(new_n191), .c(new_n192), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  aoai13aa1n02x5               g103(.a(new_n195), .b(new_n198), .c(new_n176), .d(new_n182), .o1(new_n199));
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
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n194), .c(new_n186), .d(new_n197), .o1(new_n210));
  nona22aa1n02x4               g115(.a(new_n210), .b(new_n207), .c(new_n202), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n208), .b(new_n211), .o1(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n202), .b(new_n204), .c(new_n205), .d(new_n203), .out0(new_n213));
  aoi012aa1n02x5               g118(.a(new_n204), .b(new_n202), .c(new_n205), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoi012aa1n02x5               g120(.a(new_n215), .b(new_n213), .c(new_n194), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n197), .b(new_n213), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n217), .c(new_n176), .d(new_n182), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norp02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[21] ), .b(\a[22] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n216), .clkout(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n217), .clkout(new_n229));
  aoai13aa1n02x5               g134(.a(new_n222), .b(new_n228), .c(new_n186), .d(new_n229), .o1(new_n230));
  nona22aa1n02x4               g135(.a(new_n230), .b(new_n226), .c(new_n220), .out0(new_n231));
  nanp02aa1n02x5               g136(.a(new_n227), .b(new_n231), .o1(\s[22] ));
  nano23aa1n02x4               g137(.a(new_n220), .b(new_n223), .c(new_n224), .d(new_n221), .out0(new_n233));
  nano22aa1n02x4               g138(.a(new_n198), .b(new_n213), .c(new_n233), .out0(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  aoi012aa1n02x5               g140(.a(new_n223), .b(new_n220), .c(new_n224), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n228), .c(new_n233), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n235), .c(new_n176), .d(new_n182), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  norp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n241), .c(new_n239), .d(new_n243), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n238), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n243), .b(new_n248), .c(new_n186), .d(new_n234), .o1(new_n249));
  nona22aa1n02x4               g154(.a(new_n249), .b(new_n246), .c(new_n241), .out0(new_n250));
  nanp02aa1n02x5               g155(.a(new_n247), .b(new_n250), .o1(\s[24] ));
  nanp03aa1n02x5               g156(.a(new_n194), .b(new_n209), .c(new_n206), .o1(new_n252));
  nano23aa1n02x4               g157(.a(new_n241), .b(new_n244), .c(new_n245), .d(new_n242), .out0(new_n253));
  nanp02aa1n02x5               g158(.a(new_n253), .b(new_n233), .o1(new_n254));
  160nm_fiao0012aa1n02p5x5     g159(.a(new_n244), .b(new_n241), .c(new_n245), .o(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n237), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n214), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  nano32aa1n02x4               g163(.a(new_n198), .b(new_n253), .c(new_n213), .d(new_n233), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoai13aa1n02x5               g165(.a(new_n258), .b(new_n260), .c(new_n176), .d(new_n182), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n264), .b(new_n257), .c(new_n186), .d(new_n259), .o1(new_n267));
  nona22aa1n02x4               g172(.a(new_n267), .b(new_n265), .c(new_n263), .out0(new_n268));
  nanp02aa1n02x5               g173(.a(new_n266), .b(new_n268), .o1(\s[26] ));
  norb02aa1n02x5               g174(.a(new_n264), .b(new_n265), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  nona22aa1n02x4               g176(.a(new_n229), .b(new_n254), .c(new_n271), .out0(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(\a[26] ), .clkout(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(\b[25] ), .clkout(new_n274));
  oaoi03aa1n02x5               g179(.a(new_n273), .b(new_n274), .c(new_n263), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n257), .c(new_n270), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n272), .c(new_n176), .d(new_n182), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n278), .d(new_n281), .o1(new_n283));
  nano32aa1n02x4               g188(.a(new_n246), .b(new_n243), .c(new_n225), .d(new_n222), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n217), .b(new_n284), .c(new_n270), .out0(new_n285));
  nanp02aa1n02x5               g190(.a(new_n257), .b(new_n270), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n286), .b(new_n275), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n281), .b(new_n287), .c(new_n186), .d(new_n285), .o1(new_n288));
  nona22aa1n02x4               g193(.a(new_n288), .b(new_n282), .c(new_n280), .out0(new_n289));
  nanp02aa1n02x5               g194(.a(new_n283), .b(new_n289), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g195(.clk(\a[28] ), .clkout(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(\b[27] ), .clkout(new_n292));
  oaoi03aa1n02x5               g197(.a(new_n291), .b(new_n292), .c(new_n280), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  norb02aa1n02x5               g199(.a(new_n281), .b(new_n282), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n287), .c(new_n186), .d(new_n285), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  nona22aa1n02x4               g202(.a(new_n296), .b(new_n297), .c(new_n294), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n297), .b(new_n294), .c(new_n278), .d(new_n295), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(new_n299), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oaoi03aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .o1(new_n302));
  norb03aa1n02x5               g207(.a(new_n281), .b(new_n297), .c(new_n282), .out0(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n278), .d(new_n303), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n303), .b(new_n287), .c(new_n186), .d(new_n285), .o1(new_n306));
  nona22aa1n02x4               g211(.a(new_n306), .b(new_n304), .c(new_n302), .out0(new_n307));
  nanp02aa1n02x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n303), .b(new_n304), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n287), .c(new_n186), .d(new_n285), .o1(new_n310));
  nanb02aa1n02x5               g215(.a(new_n304), .b(new_n302), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n311), .b(\b[29] ), .c(\a[30] ), .o1(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nona22aa1n02x4               g218(.a(new_n310), .b(new_n312), .c(new_n313), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n313), .b(new_n312), .c(new_n278), .d(new_n309), .o1(new_n315));
  nanp02aa1n02x5               g220(.a(new_n315), .b(new_n314), .o1(\s[31] ));
  xnbna2aa1n03x5               g221(.a(new_n101), .b(new_n104), .c(new_n107), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g225(.a(new_n114), .b(new_n109), .c(new_n115), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g227(.a(new_n119), .b(new_n116), .c(new_n109), .out0(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g229(.a(new_n123), .b(new_n323), .c(new_n111), .out0(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(new_n121), .out0(\s[8] ));
  xorb03aa1n02x5               g231(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


