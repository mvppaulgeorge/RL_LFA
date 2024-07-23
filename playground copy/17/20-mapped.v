// Benchmark "adder" written by ABC on Thu Jul 11 12:09:09 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n318, new_n319, new_n320,
    new_n322, new_n323, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n02x4               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  160nm_fiao0012aa1n02p5x5     g019(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n115));
  oabi12aa1n02x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(new_n116), .b(new_n105), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(new_n98), .clkout(new_n118));
  nanp02aa1n02x5               g023(.a(new_n100), .b(new_n99), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[5] ), .clkout(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  oaib12aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(\a[6] ), .out0(new_n122));
  oai112aa1n02x5               g027(.a(new_n118), .b(new_n119), .c(new_n102), .d(new_n122), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(new_n123), .b(new_n117), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanb02aa1n02x5               g032(.a(new_n126), .b(new_n127), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n97), .c(new_n124), .d(new_n125), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n125), .b(new_n97), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n123), .c(new_n116), .d(new_n105), .o1(new_n131));
  nona22aa1n02x4               g036(.a(new_n131), .b(new_n128), .c(new_n97), .out0(new_n132));
  nanp02aa1n02x5               g037(.a(new_n129), .b(new_n132), .o1(\s[10] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n132), .c(new_n127), .out0(\s[11] ));
  aoi013aa1n02x4               g042(.a(new_n134), .b(new_n132), .c(new_n127), .d(new_n135), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n139), .clkout(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n138), .b(new_n141), .c(new_n140), .out0(\s[12] ));
  norb02aa1n02x5               g047(.a(new_n135), .b(new_n134), .out0(new_n143));
  nanb02aa1n02x5               g048(.a(new_n139), .b(new_n141), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n144), .b(new_n128), .c(new_n130), .d(new_n143), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n123), .c(new_n116), .d(new_n105), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n134), .b(new_n141), .o1(new_n147));
  oai022aa1n02x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nano23aa1n02x4               g053(.a(new_n144), .b(new_n136), .c(new_n148), .d(new_n127), .out0(new_n149));
  nano22aa1n02x4               g054(.a(new_n149), .b(new_n140), .c(new_n147), .out0(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[13] ), .clkout(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(\b[12] ), .clkout(new_n152));
  nanp02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n153), .b(new_n154), .o1(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n146), .c(new_n150), .out0(\s[13] ));
  aoai13aa1n02x5               g061(.a(new_n153), .b(new_n155), .c(new_n146), .d(new_n150), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  norp02aa1n02x5               g066(.a(new_n161), .b(new_n155), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  aoai13aa1n02x5               g068(.a(new_n160), .b(new_n159), .c(new_n151), .d(new_n152), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n146), .d(new_n150), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  norb02aa1n02x5               g079(.a(new_n141), .b(new_n139), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n97), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n176));
  nano23aa1n02x4               g081(.a(new_n167), .b(new_n169), .c(new_n170), .d(new_n168), .out0(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n161), .c(new_n155), .out0(new_n178));
  nano32aa1n02x4               g083(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n143), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n123), .c(new_n116), .d(new_n105), .o1(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n127), .clkout(new_n181));
  norp02aa1n02x5               g086(.a(new_n126), .b(new_n97), .o1(new_n182));
  nona23aa1n02x4               g087(.a(new_n175), .b(new_n143), .c(new_n182), .d(new_n181), .out0(new_n183));
  nanp03aa1n02x5               g088(.a(new_n183), .b(new_n140), .c(new_n147), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n178), .clkout(new_n185));
  nona23aa1n02x4               g090(.a(new_n170), .b(new_n168), .c(new_n167), .d(new_n169), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n167), .b(new_n170), .o1(new_n187));
  oai122aa1n02x7               g092(.a(new_n187), .b(new_n186), .c(new_n164), .d(\b[15] ), .e(\a[16] ), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n184), .c(new_n185), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n180), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[18] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[16] ), .clkout(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(new_n192), .out0(\s[18] ));
  xroi22aa1d04x5               g101(.a(new_n193), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n194), .b(new_n193), .o1(new_n198));
  oaoi03aa1n02x5               g103(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  nona22aa1n02x4               g114(.a(new_n203), .b(new_n209), .c(new_n200), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n209), .clkout(new_n211));
  oaoi13aa1n02x5               g116(.a(new_n211), .b(new_n203), .c(\a[19] ), .d(\b[18] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n210), .b(new_n212), .out0(\s[20] ));
  nano23aa1n02x4               g118(.a(new_n200), .b(new_n207), .c(new_n208), .d(new_n201), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n197), .b(new_n214), .o1(new_n215));
  oai022aa1n02x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  oaib12aa1n02x5               g121(.a(new_n216), .b(new_n192), .c(\b[17] ), .out0(new_n217));
  nona23aa1n02x4               g122(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n218));
  aoi012aa1n02x5               g123(.a(new_n207), .b(new_n200), .c(new_n208), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n217), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n215), .c(new_n189), .d(new_n180), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[21] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  nanp03aa1n02x5               g137(.a(new_n232), .b(new_n197), .c(new_n214), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oaoi03aa1n02x5               g139(.a(new_n231), .b(new_n234), .c(new_n224), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n220), .c(new_n232), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n189), .d(new_n180), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(\s[24] ));
  and002aa1n02x5               g150(.a(new_n242), .b(new_n241), .o(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  nano32aa1n02x4               g152(.a(new_n247), .b(new_n232), .c(new_n197), .d(new_n214), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n219), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n232), .b(new_n249), .c(new_n214), .d(new_n199), .o1(new_n250));
  orn002aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  oao003aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n247), .c(new_n250), .d(new_n235), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nona22aa1n02x4               g164(.a(new_n255), .b(new_n259), .c(new_n258), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n258), .clkout(new_n261));
  aobi12aa1n02x5               g166(.a(new_n259), .b(new_n255), .c(new_n261), .out0(new_n262));
  norb02aa1n02x5               g167(.a(new_n260), .b(new_n262), .out0(\s[26] ));
  oabi12aa1n02x5               g168(.a(new_n188), .b(new_n150), .c(new_n178), .out0(new_n264));
  and002aa1n02x5               g169(.a(new_n259), .b(new_n254), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n233), .b(new_n246), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n124), .d(new_n179), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n268));
  aobi12aa1n02x5               g173(.a(new_n268), .b(new_n253), .c(new_n265), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  aobi12aa1n02x5               g178(.a(new_n270), .b(new_n269), .c(new_n267), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n266), .clkout(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n189), .c(new_n180), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n246), .b(new_n236), .c(new_n220), .d(new_n232), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n265), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n268), .b(new_n280), .c(new_n279), .d(new_n252), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n270), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n275), .b(new_n282), .c(new_n273), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n270), .b(new_n275), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n278), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n269), .c(new_n267), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n270), .b(new_n288), .c(new_n275), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n281), .c(new_n278), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n269), .c(new_n267), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n269), .c(new_n267), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n302), .b(new_n281), .c(new_n278), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g218(.a(new_n104), .b(new_n116), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g221(.a(new_n100), .b(new_n101), .out0(new_n317));
  nanb02aa1n02x5               g222(.a(new_n103), .b(new_n315), .out0(new_n318));
  oaoi13aa1n02x5               g223(.a(new_n317), .b(new_n318), .c(\a[6] ), .d(\b[5] ), .o1(new_n319));
  oai112aa1n02x5               g224(.a(new_n318), .b(new_n317), .c(\b[5] ), .d(\a[6] ), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(\s[7] ));
  nanb02aa1n02x5               g226(.a(new_n98), .b(new_n99), .out0(new_n322));
  oab012aa1n02x4               g227(.a(new_n322), .b(new_n319), .c(new_n100), .out0(new_n323));
  aoi112aa1n02x5               g228(.a(new_n319), .b(new_n100), .c(new_n118), .d(new_n99), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n323), .b(new_n324), .o1(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


