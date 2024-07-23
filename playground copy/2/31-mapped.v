// Benchmark "adder" written by ABC on Wed Jul 10 16:47:21 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n306, new_n308, new_n309, new_n311;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(\a[6] ), .clkout(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[5] ), .clkout(new_n117));
  aoi112aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n118), .b(new_n116), .c(new_n117), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n120), .b(new_n112), .c(new_n119), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n107), .c(new_n115), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  norp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nano23aa1n02x4               g036(.a(new_n128), .b(new_n130), .c(new_n131), .d(new_n129), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n130), .b(new_n128), .c(new_n131), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n127), .b(new_n133), .c(new_n134), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n125), .clkout(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n122), .clkout(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n134), .clkout(new_n138));
  aoai13aa1n02x5               g043(.a(new_n127), .b(new_n138), .c(new_n137), .d(new_n132), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n136), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n125), .b(new_n140), .c(new_n141), .d(new_n126), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n132), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n141), .b(new_n126), .c(new_n125), .d(new_n140), .out0(new_n146));
  oaoi03aa1n02x5               g051(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  oai012aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n134), .o1(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n122), .c(new_n145), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanb02aa1n02x5               g062(.a(new_n156), .b(new_n157), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n152), .b(new_n160), .c(new_n161), .d(new_n153), .out0(new_n162));
  aoi012aa1n02x5               g067(.a(new_n160), .b(new_n152), .c(new_n161), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoai13aa1n02x5               g069(.a(new_n159), .b(new_n164), .c(new_n150), .d(new_n162), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n159), .b(new_n164), .c(new_n150), .d(new_n162), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n156), .clkout(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  nanp03aa1n02x5               g076(.a(new_n165), .b(new_n168), .c(new_n171), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n171), .b(new_n165), .c(new_n168), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n156), .b(new_n169), .c(new_n170), .d(new_n157), .out0(new_n175));
  nano22aa1n02x4               g080(.a(new_n145), .b(new_n162), .c(new_n175), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n175), .b(new_n164), .c(new_n149), .d(new_n162), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n170), .b(new_n169), .c(new_n156), .o1(new_n179));
  nanp03aa1n02x5               g084(.a(new_n177), .b(new_n178), .c(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g086(.a(\b[16] ), .b(\a[17] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n182), .b(new_n180), .c(new_n183), .o1(new_n184));
  xnrb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nano23aa1n02x4               g092(.a(new_n182), .b(new_n186), .c(new_n187), .d(new_n183), .out0(new_n188));
  aoi012aa1n02x5               g093(.a(new_n186), .b(new_n182), .c(new_n187), .o1(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n189), .clkout(new_n190));
  norp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(new_n180), .d(new_n188), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n193), .b(new_n190), .c(new_n180), .d(new_n188), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  nona22aa1n02x4               g105(.a(new_n194), .b(new_n200), .c(new_n191), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n191), .clkout(new_n202));
  aobi12aa1n02x5               g107(.a(new_n200), .b(new_n194), .c(new_n202), .out0(new_n203));
  norb02aa1n02x5               g108(.a(new_n201), .b(new_n203), .out0(\s[20] ));
  nano23aa1n02x4               g109(.a(new_n191), .b(new_n198), .c(new_n199), .d(new_n192), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n188), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n206), .clkout(new_n207));
  nona23aa1n02x4               g112(.a(new_n199), .b(new_n192), .c(new_n191), .d(new_n198), .out0(new_n208));
  oaoi03aa1n02x5               g113(.a(\a[20] ), .b(\b[19] ), .c(new_n202), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  oai012aa1n02x5               g115(.a(new_n210), .b(new_n208), .c(new_n189), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[21] ), .b(\b[20] ), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n180), .d(new_n207), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n211), .c(new_n180), .d(new_n207), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  nona22aa1n02x4               g122(.a(new_n213), .b(new_n217), .c(new_n216), .out0(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n216), .clkout(new_n219));
  aobi12aa1n02x5               g124(.a(new_n217), .b(new_n213), .c(new_n219), .out0(new_n220));
  norb02aa1n02x5               g125(.a(new_n218), .b(new_n220), .out0(\s[22] ));
  nano22aa1n02x4               g126(.a(new_n206), .b(new_n212), .c(new_n217), .out0(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .c(new_n219), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n211), .c(new_n225), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  xorc02aa1n02x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n228), .c(new_n180), .d(new_n222), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n228), .c(new_n180), .d(new_n222), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  nona22aa1n02x4               g139(.a(new_n230), .b(new_n234), .c(new_n233), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n233), .clkout(new_n236));
  aobi12aa1n02x5               g141(.a(new_n234), .b(new_n230), .c(new_n236), .out0(new_n237));
  norb02aa1n02x5               g142(.a(new_n235), .b(new_n237), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n234), .b(new_n229), .o(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n225), .c(new_n205), .d(new_n188), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n225), .b(new_n209), .c(new_n205), .d(new_n190), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n226), .clkout(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n245), .b(new_n244), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(new_n242), .d(new_n243), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n180), .d(new_n241), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n180), .d(new_n241), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  nona22aa1n02x4               g158(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n252), .clkout(new_n255));
  aobi12aa1n02x5               g160(.a(new_n253), .b(new_n249), .c(new_n255), .out0(new_n256));
  norb02aa1n02x5               g161(.a(new_n254), .b(new_n256), .out0(\s[26] ));
  and002aa1n02x5               g162(.a(new_n253), .b(new_n248), .o(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  nano23aa1n02x4               g164(.a(new_n206), .b(new_n259), .c(new_n239), .d(new_n225), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(new_n180), .b(new_n260), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n262));
  aobi12aa1n02x5               g167(.a(new_n262), .b(new_n247), .c(new_n258), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n263), .out0(\s[27] ));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n266), .clkout(new_n267));
  aobi12aa1n02x5               g172(.a(new_n264), .b(new_n261), .c(new_n263), .out0(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n268), .b(new_n267), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n239), .b(new_n226), .c(new_n211), .d(new_n225), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n262), .b(new_n259), .c(new_n271), .d(new_n246), .o1(new_n272));
  aoai13aa1n02x5               g177(.a(new_n264), .b(new_n272), .c(new_n180), .d(new_n260), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n269), .b(new_n273), .c(new_n267), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n270), .o1(\s[28] ));
  norb02aa1n02x5               g180(.a(new_n264), .b(new_n269), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n272), .c(new_n180), .d(new_n260), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  aobi12aa1n02x5               g185(.a(new_n276), .b(new_n261), .c(new_n263), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n278), .c(new_n279), .out0(new_n282));
  norp02aa1n02x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g189(.a(new_n264), .b(new_n279), .c(new_n269), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n272), .c(new_n180), .d(new_n260), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n261), .c(new_n263), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n285), .b(new_n288), .out0(new_n293));
  aobi12aa1n02x5               g198(.a(new_n293), .b(new_n261), .c(new_n263), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n293), .b(new_n272), .c(new_n180), .d(new_n260), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnrb03aa1n02x5               g205(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g209(.a(new_n114), .b(new_n107), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n305), .b(\b[4] ), .c(\a[5] ), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n110), .clkout(new_n308));
  oaoi03aa1n02x5               g213(.a(new_n116), .b(new_n117), .c(new_n306), .o1(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n111), .out0(\s[7] ));
  oaoi03aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g217(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


