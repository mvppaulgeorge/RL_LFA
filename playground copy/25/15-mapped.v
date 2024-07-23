// Benchmark "adder" written by ABC on Thu Jul 11 12:43:58 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n344, new_n347,
    new_n349, new_n351;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aob012aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nanp03aa1n02x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n104), .b(new_n107), .c(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nanp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[5] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[4] ), .clkout(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\a[7] ), .clkout(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(\b[6] ), .clkout(new_n125));
  aoai13aa1n02x5               g030(.a(new_n113), .b(new_n112), .c(new_n124), .d(new_n125), .o1(new_n126));
  aobi12aa1n02x5               g031(.a(new_n126), .b(new_n116), .c(new_n123), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n119), .c(new_n110), .d(new_n111), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n97), .b(new_n98), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nano23aa1n02x4               g039(.a(new_n131), .b(new_n133), .c(new_n134), .d(new_n132), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n134), .b(new_n133), .c(new_n97), .d(new_n98), .o1(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n136), .clkout(new_n137));
  aoi012aa1n02x5               g042(.a(new_n137), .b(new_n128), .c(new_n135), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g044(.a(\a[11] ), .b(\b[10] ), .c(new_n138), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  aoi012aa1n02x5               g046(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n104), .b(new_n105), .out0(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(\a[3] ), .clkout(new_n144));
  160nm_ficinv00aa1n08x5       g049(.clk(\b[2] ), .clkout(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n146), .b(new_n108), .o1(new_n147));
  norp03aa1n02x5               g052(.a(new_n142), .b(new_n143), .c(new_n147), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n111), .clkout(new_n149));
  nona23aa1n02x4               g054(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[5] ), .b(\a[6] ), .out0(new_n151));
  xnrc02aa1n02x5               g056(.a(\b[4] ), .b(\a[5] ), .out0(new_n152));
  norp03aa1n02x5               g057(.a(new_n150), .b(new_n151), .c(new_n152), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n148), .c(new_n149), .o1(new_n154));
  norp02aa1n02x5               g059(.a(\b[10] ), .b(\a[11] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[10] ), .b(\a[11] ), .o1(new_n156));
  norp02aa1n02x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[11] ), .b(\a[12] ), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n155), .b(new_n157), .c(new_n158), .d(new_n156), .out0(new_n159));
  nanp02aa1n02x5               g064(.a(new_n159), .b(new_n135), .o1(new_n160));
  oai012aa1n02x5               g065(.a(new_n158), .b(new_n157), .c(new_n155), .o1(new_n161));
  aobi12aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n137), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n154), .d(new_n127), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n167));
  xnrb03aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n02x4               g073(.a(new_n111), .b(new_n142), .c(new_n143), .d(new_n147), .o1(new_n169));
  oaib12aa1n02x5               g074(.a(new_n126), .b(new_n150), .c(new_n123), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n160), .clkout(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n170), .c(new_n169), .d(new_n153), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nona23aa1n02x4               g079(.a(new_n174), .b(new_n166), .c(new_n165), .d(new_n173), .out0(new_n175));
  oai012aa1n02x5               g080(.a(new_n174), .b(new_n173), .c(new_n165), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n175), .c(new_n172), .d(new_n162), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n181), .clkout(new_n182));
  norp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n179), .c(new_n177), .d(new_n182), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n179), .clkout(new_n188));
  nano23aa1n02x4               g093(.a(new_n165), .b(new_n173), .c(new_n174), .d(new_n166), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n176), .clkout(new_n190));
  aoai13aa1n02x5               g095(.a(new_n182), .b(new_n190), .c(new_n163), .d(new_n189), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n185), .b(new_n191), .c(new_n188), .o1(new_n192));
  norp02aa1n02x5               g097(.a(new_n192), .b(new_n187), .o1(\s[16] ));
  nona22aa1n02x4               g098(.a(new_n189), .b(new_n185), .c(new_n181), .out0(new_n194));
  norp02aa1n02x5               g099(.a(new_n194), .b(new_n160), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n170), .c(new_n169), .d(new_n153), .o1(new_n196));
  nona23aa1n02x4               g101(.a(new_n158), .b(new_n156), .c(new_n155), .d(new_n157), .out0(new_n197));
  oai012aa1n02x5               g102(.a(new_n161), .b(new_n197), .c(new_n136), .o1(new_n198));
  nona23aa1n02x4               g103(.a(new_n184), .b(new_n180), .c(new_n179), .d(new_n183), .out0(new_n199));
  norp02aa1n02x5               g104(.a(new_n199), .b(new_n175), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n184), .b(new_n183), .c(new_n179), .o1(new_n201));
  oai012aa1n02x5               g106(.a(new_n201), .b(new_n199), .c(new_n176), .o1(new_n202));
  aoi012aa1n02x5               g107(.a(new_n202), .b(new_n198), .c(new_n200), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n196), .c(new_n203), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g110(.clk(\a[17] ), .clkout(new_n206));
  nanb02aa1n02x5               g111(.a(\b[16] ), .b(new_n206), .out0(new_n207));
  oabi12aa1n02x5               g112(.a(new_n202), .b(new_n162), .c(new_n194), .out0(new_n208));
  aoai13aa1n02x5               g113(.a(new_n204), .b(new_n208), .c(new_n128), .d(new_n195), .o1(new_n209));
  xnrc02aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n209), .c(new_n207), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g116(.clk(\a[18] ), .clkout(new_n212));
  xroi22aa1d04x5               g117(.a(new_n206), .b(\b[16] ), .c(new_n212), .d(\b[17] ), .out0(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  oai022aa1n02x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  oaib12aa1n02x5               g120(.a(new_n215), .b(new_n212), .c(\b[17] ), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n196), .d(new_n203), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  norp02aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  aoi112aa1n02x5               g132(.a(new_n220), .b(new_n227), .c(new_n217), .d(new_n223), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n220), .clkout(new_n229));
  nona23aa1n02x4               g134(.a(new_n189), .b(new_n135), .c(new_n199), .d(new_n197), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n203), .b(new_n230), .c(new_n154), .d(new_n127), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n216), .clkout(new_n232));
  aoai13aa1n02x5               g137(.a(new_n223), .b(new_n232), .c(new_n231), .d(new_n213), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n226), .b(new_n233), .c(new_n229), .o1(new_n234));
  norp02aa1n02x5               g139(.a(new_n234), .b(new_n228), .o1(\s[20] ));
  nona23aa1n02x4               g140(.a(new_n227), .b(new_n204), .c(new_n210), .d(new_n222), .out0(new_n236));
  nona23aa1n02x4               g141(.a(new_n225), .b(new_n221), .c(new_n220), .d(new_n224), .out0(new_n237));
  oaoi03aa1n02x5               g142(.a(\a[20] ), .b(\b[19] ), .c(new_n229), .o1(new_n238));
  oabi12aa1n02x5               g143(.a(new_n238), .b(new_n237), .c(new_n216), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n236), .c(new_n196), .d(new_n203), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  aoi112aa1n02x5               g152(.a(new_n243), .b(new_n247), .c(new_n241), .d(new_n245), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n243), .clkout(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n236), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n245), .b(new_n239), .c(new_n231), .d(new_n250), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n246), .b(new_n251), .c(new_n249), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n252), .b(new_n248), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n237), .clkout(new_n254));
  norp02aa1n02x5               g159(.a(new_n246), .b(new_n244), .o1(new_n255));
  nanp03aa1n02x5               g160(.a(new_n213), .b(new_n254), .c(new_n255), .o1(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n239), .c(new_n255), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(new_n196), .d(new_n203), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  nanb02aa1n02x5               g167(.a(new_n261), .b(new_n262), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  norp02aa1n02x5               g169(.a(\b[23] ), .b(\a[24] ), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nanb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  aoi112aa1n02x5               g173(.a(new_n261), .b(new_n268), .c(new_n259), .d(new_n264), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n261), .clkout(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n256), .clkout(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n258), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n264), .b(new_n272), .c(new_n231), .d(new_n271), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n267), .b(new_n273), .c(new_n270), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n269), .o1(\s[24] ));
  nona23aa1n02x4               g180(.a(new_n266), .b(new_n262), .c(new_n261), .d(new_n265), .out0(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  nano22aa1n02x4               g182(.a(new_n236), .b(new_n255), .c(new_n277), .out0(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n278), .clkout(new_n279));
  oai012aa1n02x5               g184(.a(new_n266), .b(new_n265), .c(new_n261), .o1(new_n280));
  aobi12aa1n02x5               g185(.a(new_n280), .b(new_n277), .c(new_n257), .out0(new_n281));
  nona32aa1n02x4               g186(.a(new_n239), .b(new_n276), .c(new_n246), .d(new_n244), .out0(new_n282));
  nanp02aa1n02x5               g187(.a(new_n282), .b(new_n281), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n196), .d(new_n203), .o1(new_n285));
  xorb03aa1n02x5               g190(.a(new_n285), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[25] ), .b(\b[24] ), .out0(new_n288));
  xorc02aa1n02x5               g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  aoi112aa1n02x5               g194(.a(new_n287), .b(new_n289), .c(new_n285), .d(new_n288), .o1(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n287), .clkout(new_n291));
  aoai13aa1n02x5               g196(.a(new_n288), .b(new_n283), .c(new_n231), .d(new_n278), .o1(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n289), .clkout(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n292), .c(new_n291), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n290), .o1(\s[26] ));
  and002aa1n02x5               g200(.a(new_n289), .b(new_n288), .o(new_n296));
  nano22aa1n02x4               g201(.a(new_n256), .b(new_n296), .c(new_n277), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n208), .c(new_n128), .d(new_n195), .o1(new_n298));
  oaoi03aa1n02x5               g203(.a(\a[26] ), .b(\b[25] ), .c(new_n291), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n283), .c(new_n296), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  xnbna2aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n298), .out0(\s[27] ));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n303), .clkout(new_n304));
  160nm_ficinv00aa1n08x5       g209(.clk(new_n301), .clkout(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n300), .c(new_n298), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n306), .b(new_n304), .c(new_n307), .out0(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n296), .clkout(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(new_n299), .clkout(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n309), .c(new_n282), .d(new_n281), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n301), .b(new_n311), .c(new_n231), .d(new_n297), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n307), .b(new_n312), .c(new_n304), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n308), .o1(\s[28] ));
  norb02aa1n02x5               g219(.a(new_n301), .b(new_n307), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n311), .c(new_n231), .d(new_n297), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[28] ), .b(\a[29] ), .out0(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  160nm_ficinv00aa1n08x5       g224(.clk(new_n315), .clkout(new_n320));
  aoi012aa1n02x5               g225(.a(new_n320), .b(new_n300), .c(new_n298), .o1(new_n321));
  nano22aa1n02x4               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  norp02aa1n02x5               g227(.a(new_n319), .b(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g229(.a(new_n301), .b(new_n318), .c(new_n307), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n311), .c(new_n231), .d(new_n297), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .out0(new_n328));
  aoi012aa1n02x5               g233(.a(new_n328), .b(new_n326), .c(new_n327), .o1(new_n329));
  160nm_ficinv00aa1n08x5       g234(.clk(new_n325), .clkout(new_n330));
  aoi012aa1n02x5               g235(.a(new_n330), .b(new_n300), .c(new_n298), .o1(new_n331));
  nano22aa1n02x4               g236(.a(new_n331), .b(new_n327), .c(new_n328), .out0(new_n332));
  norp02aa1n02x5               g237(.a(new_n329), .b(new_n332), .o1(\s[30] ));
  nona32aa1n02x4               g238(.a(new_n301), .b(new_n328), .c(new_n318), .d(new_n307), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n300), .c(new_n298), .o1(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n327), .carry(new_n336));
  xnrc02aa1n02x5               g241(.a(\b[30] ), .b(\a[31] ), .out0(new_n337));
  nano22aa1n02x4               g242(.a(new_n335), .b(new_n336), .c(new_n337), .out0(new_n338));
  160nm_ficinv00aa1n08x5       g243(.clk(new_n334), .clkout(new_n339));
  aoai13aa1n02x5               g244(.a(new_n339), .b(new_n311), .c(new_n231), .d(new_n297), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n337), .b(new_n340), .c(new_n336), .o1(new_n341));
  norp02aa1n02x5               g246(.a(new_n341), .b(new_n338), .o1(\s[31] ));
  xnbna2aa1n03x5               g247(.a(new_n142), .b(new_n108), .c(new_n146), .out0(\s[3] ));
  oaoi03aa1n02x5               g248(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n344));
  xorb03aa1n02x5               g249(.a(new_n344), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g250(.a(new_n169), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g251(.a(new_n120), .b(new_n121), .c(new_n169), .o1(new_n347));
  xnrc02aa1n02x5               g252(.a(new_n347), .b(new_n117), .out0(\s[6] ));
  oaoi03aa1n02x5               g253(.a(\a[6] ), .b(\b[5] ), .c(new_n347), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g255(.a(new_n124), .b(new_n125), .c(new_n349), .o1(new_n351));
  xnrb03aa1n02x5               g256(.a(new_n351), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g257(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


