// Benchmark "adder" written by ABC on Thu Jul 11 11:34:54 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n310, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
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
  oao003aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .carry(new_n122));
  160nm_fiao0012aa1n02p5x5     g027(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n123));
  oabi12aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n114), .out0(new_n124));
  nanb02aa1n02x5               g029(.a(new_n124), .b(new_n118), .out0(new_n125));
  xnrc02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n126), .clkout(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n125), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  norp02aa1n02x5               g038(.a(new_n133), .b(new_n126), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(\b[9] ), .clkout(new_n136));
  oaoi03aa1n02x5               g041(.a(new_n97), .b(new_n136), .c(new_n98), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n132), .b(new_n135), .c(new_n137), .out0(\s[11] ));
  aob012aa1n02x5               g043(.a(new_n132), .b(new_n135), .c(new_n137), .out0(new_n139));
  oai012aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n143), .b(new_n131), .c(new_n130), .d(new_n142), .out0(new_n144));
  norp03aa1n02x5               g049(.a(new_n144), .b(new_n133), .c(new_n126), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n142), .b(new_n130), .c(new_n143), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n137), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n146), .c(new_n149), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g058(.clk(\a[13] ), .clkout(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(\b[12] ), .clkout(new_n155));
  nanp02aa1n02x5               g060(.a(new_n146), .b(new_n149), .o1(new_n156));
  oaoi03aa1n02x5               g061(.a(new_n154), .b(new_n155), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n02x4               g065(.a(new_n150), .b(new_n159), .c(new_n160), .d(new_n151), .out0(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n160), .b(new_n159), .c(new_n154), .d(new_n155), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n146), .d(new_n149), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xorc02aa1n02x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xorc02aa1n02x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n130), .b(new_n142), .c(new_n143), .d(new_n131), .out0(new_n172));
  nanp03aa1n02x5               g077(.a(new_n161), .b(new_n167), .c(new_n168), .o1(new_n173));
  nano22aa1n02x4               g078(.a(new_n173), .b(new_n134), .c(new_n172), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n175));
  nanb03aa1n02x5               g080(.a(new_n163), .b(new_n168), .c(new_n167), .out0(new_n176));
  oab012aa1n02x4               g081(.a(new_n166), .b(\a[16] ), .c(\b[15] ), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n176), .b(new_n177), .c(\b[15] ), .d(\a[16] ), .o1(new_n178));
  aoib12aa1n02x5               g083(.a(new_n178), .b(new_n148), .c(new_n173), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n175), .b(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[18] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[17] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\b[16] ), .clkout(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  xroi22aa1d04x5               g091(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n184), .b(new_n183), .o1(new_n188));
  oaoi03aa1n02x5               g093(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n180), .d(new_n187), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n189), .c(new_n180), .d(new_n187), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nona22aa1n02x4               g104(.a(new_n193), .b(new_n199), .c(new_n190), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n190), .clkout(new_n201));
  aobi12aa1n02x5               g106(.a(new_n199), .b(new_n193), .c(new_n201), .out0(new_n202));
  norb02aa1n02x5               g107(.a(new_n200), .b(new_n202), .out0(\s[20] ));
  nano23aa1n02x4               g108(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n191), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n187), .b(new_n204), .o1(new_n205));
  oai022aa1n02x5               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  oaib12aa1n02x5               g111(.a(new_n206), .b(new_n182), .c(\b[17] ), .out0(new_n207));
  nona23aa1n02x4               g112(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n208));
  aoi012aa1n02x5               g113(.a(new_n197), .b(new_n190), .c(new_n198), .o1(new_n209));
  oai012aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n207), .o1(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n205), .c(new_n175), .d(new_n179), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g124(.clk(\a[21] ), .clkout(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(\a[22] ), .clkout(new_n221));
  xroi22aa1d04x5               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n187), .c(new_n204), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\b[21] ), .clkout(new_n224));
  oaoi03aa1n02x5               g129(.a(new_n221), .b(new_n224), .c(new_n214), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n222), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n227), .b(new_n223), .c(new_n175), .d(new_n179), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  xorc02aa1n02x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  and002aa1n02x5               g140(.a(new_n232), .b(new_n231), .o(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n222), .c(new_n187), .d(new_n204), .out0(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n209), .clkout(new_n239));
  aoai13aa1n02x5               g144(.a(new_n222), .b(new_n239), .c(new_n204), .d(new_n189), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n241));
  oab012aa1n02x4               g146(.a(new_n241), .b(\a[24] ), .c(\b[23] ), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .d(new_n225), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n180), .c(new_n238), .o1(new_n244));
  xnrc02aa1n02x5               g149(.a(\b[24] ), .b(\a[25] ), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n245), .clkout(new_n246));
  xnrc02aa1n02x5               g151(.a(new_n244), .b(new_n246), .out0(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n246), .b(new_n243), .c(new_n180), .d(new_n238), .o1(new_n250));
  xnrc02aa1n02x5               g155(.a(\b[25] ), .b(\a[26] ), .out0(new_n251));
  nanp03aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n251), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n249), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(\s[26] ));
  norp02aa1n02x5               g159(.a(new_n251), .b(new_n245), .o1(new_n255));
  nano22aa1n02x4               g160(.a(new_n223), .b(new_n236), .c(new_n255), .out0(new_n256));
  nanp02aa1n02x5               g161(.a(new_n180), .b(new_n256), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .c(new_n249), .carry(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n243), .c(new_n255), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n257), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n261), .clkout(new_n265));
  aoi012aa1n02x5               g170(.a(new_n265), .b(new_n257), .c(new_n260), .o1(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[27] ), .b(\a[28] ), .out0(new_n267));
  nano22aa1n02x4               g172(.a(new_n266), .b(new_n264), .c(new_n267), .out0(new_n268));
  aobi12aa1n02x5               g173(.a(new_n256), .b(new_n175), .c(new_n179), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n236), .b(new_n226), .c(new_n210), .d(new_n222), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n255), .clkout(new_n271));
  aoai13aa1n02x5               g176(.a(new_n258), .b(new_n271), .c(new_n270), .d(new_n242), .o1(new_n272));
  oai012aa1n02x5               g177(.a(new_n261), .b(new_n272), .c(new_n269), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n267), .b(new_n273), .c(new_n264), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n268), .o1(\s[28] ));
  norb02aa1n02x5               g180(.a(new_n261), .b(new_n267), .out0(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n257), .c(new_n260), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n278), .b(new_n279), .c(new_n280), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n276), .b(new_n272), .c(new_n269), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n280), .b(new_n282), .c(new_n279), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n261), .b(new_n280), .c(new_n267), .out0(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n257), .c(new_n260), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n286), .b(new_n272), .c(new_n269), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n291), .o1(\s[30] ));
  norb02aa1n02x5               g199(.a(new_n286), .b(new_n290), .out0(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n295), .clkout(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n257), .c(new_n260), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n295), .b(new_n272), .c(new_n269), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g212(.a(new_n119), .b(new_n120), .c(new_n109), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g214(.a(new_n116), .b(new_n115), .o1(new_n310));
  aobi12aa1n02x5               g215(.a(new_n122), .b(new_n109), .c(new_n310), .out0(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


