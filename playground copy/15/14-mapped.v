// Benchmark "adder" written by ABC on Thu Jul 11 11:59:18 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n331, new_n333, new_n335;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n02x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
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
  aoi112aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  aoib12aa1n02x5               g022(.a(new_n117), .b(new_n116), .c(\b[5] ), .out0(new_n118));
  oai012aa1n02x5               g023(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(new_n112), .c(new_n118), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n120), .b(new_n107), .c(new_n115), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nano23aa1n02x4               g032(.a(new_n124), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n128), .clkout(new_n129));
  aoi012aa1n02x5               g034(.a(new_n124), .b(new_n126), .c(new_n125), .o1(new_n130));
  oai012aa1n02x5               g035(.a(new_n130), .b(new_n121), .c(new_n129), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n133), .out0(new_n136));
  nanp02aa1n02x5               g041(.a(new_n131), .b(new_n136), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n134), .out0(\s[12] ));
  nona23aa1n02x4               g046(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n138), .out0(new_n142));
  oaoi03aa1n02x5               g047(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n143), .clkout(new_n144));
  oai012aa1n02x5               g049(.a(new_n144), .b(new_n142), .c(new_n130), .o1(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  oai013aa1n02x4               g051(.a(new_n146), .b(new_n121), .c(new_n129), .d(new_n142), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoi112aa1n02x5               g057(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n153));
  oai112aa1n02x5               g058(.a(new_n136), .b(new_n140), .c(new_n153), .d(new_n124), .o1(new_n154));
  norp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1n02x4               g061(.a(new_n149), .b(new_n155), .c(new_n156), .d(new_n150), .out0(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  aoi012aa1n02x5               g063(.a(new_n155), .b(new_n149), .c(new_n156), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n154), .d(new_n144), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n121), .clkout(new_n162));
  nona32aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n142), .d(new_n129), .out0(new_n163));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n163), .c(new_n161), .out0(\s[15] ));
  aoi012aa1n02x5               g072(.a(new_n166), .b(new_n163), .c(new_n161), .o1(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  norp03aa1n02x5               g076(.a(new_n168), .b(new_n171), .c(new_n164), .o1(new_n172));
  oai012aa1n02x5               g077(.a(new_n171), .b(new_n168), .c(new_n164), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n133), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n164), .b(new_n169), .c(new_n170), .d(new_n165), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n157), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n177), .b(new_n128), .c(new_n175), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n159), .clkout(new_n180));
  aoai13aa1n02x5               g085(.a(new_n176), .b(new_n180), .c(new_n145), .d(new_n157), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n170), .b(new_n169), .c(new_n164), .o1(new_n182));
  nanp03aa1n02x5               g087(.a(new_n179), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp02aa1n02x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  norp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  aobi12aa1n02x5               g093(.a(new_n182), .b(new_n160), .c(new_n176), .out0(new_n189));
  oai112aa1n02x5               g094(.a(new_n189), .b(new_n179), .c(\b[16] ), .d(\a[17] ), .o1(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n188), .b(new_n190), .c(new_n185), .out0(\s[18] ));
  norp02aa1n02x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  nano23aa1n02x4               g097(.a(new_n192), .b(new_n186), .c(new_n187), .d(new_n185), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  aoi012aa1n02x5               g099(.a(new_n186), .b(new_n192), .c(new_n187), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n189), .d(new_n179), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  aoi112aa1n02x5               g108(.a(new_n199), .b(new_n203), .c(new_n196), .d(new_n200), .o1(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n199), .clkout(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n195), .clkout(new_n206));
  norb02aa1n02x5               g111(.a(new_n200), .b(new_n199), .out0(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n183), .d(new_n193), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n203), .clkout(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n205), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n204), .o1(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n199), .b(new_n201), .c(new_n202), .d(new_n200), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n193), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n214));
  oai112aa1n02x5               g119(.a(new_n207), .b(new_n203), .c(new_n214), .d(new_n186), .o1(new_n215));
  oai012aa1n02x5               g120(.a(new_n202), .b(new_n201), .c(new_n199), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n213), .c(new_n189), .d(new_n179), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xnrc02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  aoi112aa1n02x5               g129(.a(new_n221), .b(new_n224), .c(new_n219), .d(new_n222), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n221), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n213), .clkout(new_n227));
  aoai13aa1n02x5               g132(.a(new_n222), .b(new_n217), .c(new_n183), .d(new_n227), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n223), .b(new_n228), .c(new_n226), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n225), .o1(\s[22] ));
  nanb02aa1n02x5               g135(.a(new_n223), .b(new_n222), .out0(new_n231));
  nano22aa1n02x4               g136(.a(new_n231), .b(new_n193), .c(new_n212), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  norb02aa1n02x5               g138(.a(new_n222), .b(new_n223), .out0(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n217), .c(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n189), .d(new_n179), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n239), .clkout(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n236), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n183), .d(new_n232), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n241), .clkout(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n243), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n247), .b(new_n242), .o1(\s[24] ));
  nanp02aa1n02x5               g153(.a(new_n241), .b(new_n240), .o1(new_n249));
  norp03aa1n02x5               g154(.a(new_n213), .b(new_n231), .c(new_n249), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n249), .clkout(new_n252));
  norp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n254));
  nanp03aa1n02x5               g159(.a(new_n235), .b(new_n240), .c(new_n241), .o1(new_n255));
  nona22aa1n02x4               g160(.a(new_n255), .b(new_n254), .c(new_n253), .out0(new_n256));
  aoi013aa1n02x4               g161(.a(new_n256), .b(new_n217), .c(new_n252), .d(new_n234), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n251), .c(new_n189), .d(new_n179), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n260), .clkout(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n257), .clkout(new_n265));
  aoai13aa1n02x5               g170(.a(new_n261), .b(new_n265), .c(new_n183), .d(new_n250), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n262), .clkout(new_n267));
  aoi012aa1n02x5               g172(.a(new_n267), .b(new_n266), .c(new_n264), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n263), .o1(\s[26] ));
  and002aa1n02x5               g174(.a(new_n262), .b(new_n261), .o(new_n270));
  nano23aa1n02x4               g175(.a(new_n213), .b(new_n249), .c(new_n270), .d(new_n234), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  aoi112aa1n02x5               g177(.a(new_n231), .b(new_n249), .c(new_n215), .d(new_n216), .o1(new_n273));
  oaoi03aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .o1(new_n274));
  oaoi13aa1n02x5               g179(.a(new_n274), .b(new_n270), .c(new_n273), .d(new_n256), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n272), .c(new_n189), .d(new_n179), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .out0(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n256), .clkout(new_n280));
  nanp03aa1n02x5               g185(.a(new_n217), .b(new_n234), .c(new_n252), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n270), .clkout(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n274), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n283), .b(new_n282), .c(new_n280), .d(new_n281), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  aoi112aa1n02x5               g190(.a(new_n285), .b(new_n284), .c(new_n183), .d(new_n271), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n286), .b(new_n278), .c(new_n279), .out0(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n130), .clkout(new_n288));
  aoai13aa1n02x5               g193(.a(new_n157), .b(new_n143), .c(new_n175), .d(new_n288), .o1(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n176), .clkout(new_n290));
  aoai13aa1n02x5               g195(.a(new_n182), .b(new_n290), .c(new_n289), .d(new_n159), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n271), .b(new_n291), .c(new_n162), .d(new_n178), .o1(new_n292));
  nona22aa1n02x4               g197(.a(new_n292), .b(new_n284), .c(new_n285), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n279), .b(new_n293), .c(new_n278), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n287), .o1(\s[28] ));
  nano22aa1n02x4               g200(.a(new_n285), .b(new_n279), .c(new_n278), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n284), .c(new_n183), .d(new_n271), .o1(new_n297));
  160nm_ficinv00aa1n08x5       g202(.clk(\a[28] ), .clkout(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(\b[27] ), .clkout(new_n299));
  oaoi03aa1n02x5               g204(.a(new_n298), .b(new_n299), .c(new_n285), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .out0(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n301), .clkout(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n297), .c(new_n300), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n300), .clkout(new_n304));
  aoi112aa1n02x5               g209(.a(new_n301), .b(new_n304), .c(new_n276), .d(new_n296), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g212(.a(new_n302), .b(new_n285), .c(new_n279), .d(new_n278), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n284), .c(new_n183), .d(new_n271), .o1(new_n309));
  oaoi03aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n310));
  160nm_ficinv00aa1n08x5       g215(.clk(new_n310), .clkout(new_n311));
  xorc02aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  160nm_ficinv00aa1n08x5       g217(.clk(new_n312), .clkout(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n309), .c(new_n311), .o1(new_n314));
  aoi112aa1n02x5               g219(.a(new_n312), .b(new_n310), .c(new_n276), .d(new_n308), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n314), .b(new_n315), .o1(\s[30] ));
  and003aa1n02x5               g221(.a(new_n296), .b(new_n312), .c(new_n301), .o(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  aoi112aa1n02x5               g224(.a(new_n319), .b(new_n318), .c(new_n276), .d(new_n317), .o1(new_n320));
  aoai13aa1n02x5               g225(.a(new_n317), .b(new_n284), .c(new_n183), .d(new_n271), .o1(new_n321));
  160nm_ficinv00aa1n08x5       g226(.clk(new_n318), .clkout(new_n322));
  160nm_ficinv00aa1n08x5       g227(.clk(new_n319), .clkout(new_n323));
  aoi012aa1n02x5               g228(.a(new_n323), .b(new_n321), .c(new_n322), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n324), .b(new_n320), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai122aa1n02x7               g234(.a(new_n106), .b(new_n105), .c(new_n100), .d(\b[4] ), .e(\a[5] ), .o1(new_n330));
  aob012aa1n02x5               g235(.a(new_n330), .b(\b[4] ), .c(\a[5] ), .out0(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[5] ), .c(new_n116), .out0(\s[6] ));
  oaoi03aa1n02x5               g237(.a(\a[6] ), .b(\b[5] ), .c(new_n331), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g239(.a(new_n110), .b(new_n333), .c(new_n111), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g241(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


